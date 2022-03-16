/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "config.h"
#include "critical.h"
#include "libc.h"

#include "early_init.h"

#include "stm32f1xx_it.c.h"
#include "build/gitversion.h"

#define F_ACC_CONTROL         (1 << 0)
#define F_ACC_INIT_MAGIC      (1 << 1)
#define F_ACC_SPEED_LOCKOUT   (1 << 2)
#define F_LOW_SPEED_LEAD      (1 << 3)
#define F_FORCE_PASSTHRU      (1 << 4)
#define CONFIG_PAGE_ADDRESS   0x803F800

uint16_t features = (F_ACC_CONTROL | F_ACC_INIT_MAGIC | F_ACC_SPEED_LOCKOUT | F_LOW_SPEED_LEAD);

// 10 msg
#define MAX_ACC_CONTROL_TIMEOUT 300
// 10 msg
#define MAX_AEB_CONTROL_TIMEOUT 500

// mute for 5 seconds after any aeb msg
#define MAX_AEB_TIMEOUT         5000

#define CRASH_STATE_RESET       0
#define CRASH_STATE_PENDING     1
#define CRASH_STATE_PASSTHRU    2

#define CRASH_THRS_PASSTHRU     10

#define CAN_FILTER_SIZE         8

// input
#define CAN_FILTER_INPUT_MAGIC      0x2A0U  // 672
#define CAN_FILTER_ISOTP_RX         0x2A1U  // 673
#define CAN_FILTER_ACC_CONTROL      0x2A2U  // 674
#define CAN_FILTER_PRE_COLLISION_2  0x2A3U  // 684

// output
#define CAN_FILTER_STATE            0x2A8U  // 680
#define CAN_FILTER_ISOTP_TX         0x2A9U  // 681
#define CAN_FILTER_ACC_CONTROL_COPY 0x2AAU  // 682

void __initialize_hardware_early(void) {
  early_initialization();
}

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

// global CAN stats
uint32_t can_rx_errs = 0;
uint32_t can_send_errs = 0;

uint32_t can_rx_cnt = 0;
uint32_t can_tx_cnt = 0;
uint32_t can_txd_cnt = 0;
uint32_t can_err_cnt = 0;
uint32_t can_overflow_cnt = 0;

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
TIM_HandleTypeDef htim6;

IWDG_HandleTypeDef hiwdg;

uint8_t low_speed_lockout = 3;
uint16_t vehicle_speed = 0;
float cruise_active = 0.0f;

uint32_t acc_control_timeout = MAX_ACC_CONTROL_TIMEOUT;
uint32_t pre_collision_timeout = MAX_AEB_CONTROL_TIMEOUT;
uint32_t pre_collision_2_timeout = MAX_AEB_CONTROL_TIMEOUT;
uint8_t pre_collision_2_data[8];
bool pre_collision_2_present = false;

uint32_t aeb_timeout = MAX_AEB_TIMEOUT;

uint8_t status_tick_count = 0;
bool debug_ring_ready = false;

uint8_t crash_state;

/* ISO TP */
#define ISOTP_BUF_SIZE 0x110

uint8_t isotp_rx_buf[ISOTP_BUF_SIZE];
uint8_t* isotp_rx_ptr = NULL;
int isotp_rx_remain = 0;

uint8_t isotp_tx_buf[ISOTP_BUF_SIZE];
uint8_t* isotp_tx_ptr = NULL;
int isotp_tx_remain = 0;
int isotp_tx_index = 0;

extern uint32_t reserved_sram[4];
extern int _app_start[];

/* USER CODE BEGIN PV */
// ********************* Critical section helpers *********************

// debug uart
#define DEBUG_FIFO_SIZE 0x400U

typedef struct ring_buffer {
  volatile uint16_t w_ptr_tx;
  volatile uint16_t r_ptr_tx;
  uint8_t *elems_tx;
  uint32_t tx_fifo_size;
} ring_buffer;

#define UART_BUFFER(x, size_tx) \
  uint8_t elems_tx_##x[size_tx]; \
  ring_buffer ring_buffer_##x = {  \
    .w_ptr_tx = 0, \
    .r_ptr_tx = 0, \
    .elems_tx = ((uint8_t *)&(elems_tx_##x)), \
    .tx_fifo_size = (size_tx), \
  };

// ***************************** Function prototypes *****************************
void reset_crash_state()
{
  reserved_sram[1] = 0xDEADBE00;
  crash_state = CRASH_STATE_RESET;
}

void update_crash_state()
{
  uint8_t crashes = (reserved_sram[1] & 0xFF);
  // silent
  if (crashes >= CRASH_THRS_PASSTHRU)
  {
    crash_state = CRASH_STATE_PASSTHRU;
  }
  else
  {
    crash_state = CRASH_STATE_PENDING;
  }
}

// ******************************** UART buffers ********************************
UART_BUFFER(debug, DEBUG_FIFO_SIZE)

void clear_ring_buffer(ring_buffer *q) {
  ENTER_CRITICAL();
  q->w_ptr_tx = 0;
  q->r_ptr_tx = 0;
  EXIT_CRITICAL();
}

static void dbg_putc(ring_buffer *q, char elem) {
  if (!debug_ring_ready)
  {
    return;
  }

  ENTER_CRITICAL();
  q->elems_tx[q->w_ptr_tx] = elem;
  q->w_ptr_tx = (q->w_ptr_tx + 1U) % q->tx_fifo_size;
  if (q->w_ptr_tx == q->r_ptr_tx) {
    q->r_ptr_tx = (q->r_ptr_tx + 1U) % q->tx_fifo_size;
  }
  EXIT_CRITICAL();
  return;
}

void dbg_putch(const char a) {
  // do not lock the cpu
  //while (!dbg_putc(&ring_buffer_debug, a));
  dbg_putc(&ring_buffer_debug, a);
}

void dbg_puts(const char *a) {
  for (const char *in = a; *in; in++) {
    if (*in == '\n') dbg_putch('\r');
    dbg_putch(*in);
  }
}

void putui(uint32_t i) {
  uint32_t i_copy = i;
  char str[11];
  uint8_t idx = 10;
  str[idx] = '\0';
  idx--;
  do {
    str[idx] = (i_copy % 10U) + 0x30U;
    idx--;
    i_copy /= 10;
  } while (i_copy != 0U);
  dbg_puts(&str[idx + 1U]);
}

void dbg_ts_puts(const char *a) {
  putui(HAL_GetTick());
  dbg_puts(a);
}

void puth(unsigned int i) {
  const char c[] = "0123456789abcdef";
  for (int pos = 28; pos != -4; pos -= 4) {
    dbg_putch(c[(i >> (unsigned int)(pos)) & 0xFU]);
  }
}

void puth2(unsigned int i) {
  const char c[] = "0123456789abcdef";
  for (int pos = 4; pos != -4; pos -= 4) {
    dbg_putch(c[(i >> (unsigned int)(pos)) & 0xFU]);
  }
}

void hexdump(const void *a, int l) {
  if (a != NULL) {
    for (int i=0; i < l; i++) {
      if ((i != 0) && ((i & 0xf) == 0)) dbg_puts("\n");
      puth2(((const unsigned char*)a)[i]);
      dbg_puts(" ");
    }
  }
  dbg_puts("\n");
}

typedef struct {
  uint16_t Id;
  uint8_t Size;
  uint8_t Data[8];
} CANMessage;

typedef struct {
  volatile uint32_t w_ptr;
  volatile uint32_t r_ptr;
  uint32_t fifo_size;
  CANMessage *elems;
} can_ring;

#define can_buffer(x, size) \
  CANMessage elems_##x[size]; \
  can_ring can_##x = { .w_ptr = 0, .r_ptr = 0, .fifo_size = (size), .elems = (CANMessage*)&(elems_##x) };

can_buffer(tx1_q, 0x100)
can_buffer(tx2_q, 0x100)

can_ring *can_queues[] = {&can_tx1_q, &can_tx2_q};

// cache for about 0.5 second
can_buffer(acc_control_q, 0x10)

// ********************* interrupt safe queue *********************
bool can_pop(can_ring *q, CANMessage *elem) {
  bool ret = 0;

  ENTER_CRITICAL();
  if (q->w_ptr != q->r_ptr) {
    *elem = q->elems[q->r_ptr];
    if ((q->r_ptr + 1U) == q->fifo_size) {
      q->r_ptr = 0;
    } else {
      q->r_ptr += 1U;
    }
    ret = 1;
  }
  EXIT_CRITICAL();

  return ret;
}

bool can_push(can_ring *q, CANMessage *elem) {
  bool ret = false;
  uint32_t next_w_ptr;

  ENTER_CRITICAL();
  if ((q->w_ptr + 1U) == q->fifo_size) {
    next_w_ptr = 0;
  } else {
    next_w_ptr = q->w_ptr + 1U;
  }
  if (next_w_ptr != q->r_ptr) {
    q->elems[q->w_ptr] = *elem;
    q->w_ptr = next_w_ptr;
    ret = true;
  }
  EXIT_CRITICAL();
  if (!ret) {
    can_overflow_cnt++;
    #ifdef DEBUG
      dbg_puts("can_push failed!\n");
    #endif
  }
  return ret;
}

uint32_t can_slots_empty(can_ring *q) {
  uint32_t ret = 0;

  ENTER_CRITICAL();
  if (q->w_ptr >= q->r_ptr) {
    ret = q->fifo_size - 1U - q->w_ptr + q->r_ptr;
  } else {
    ret = q->r_ptr - q->w_ptr - 1U;
  }
  EXIT_CRITICAL();

  return ret;
}

void can_clear(can_ring *q) {
  ENTER_CRITICAL();
  q->w_ptr = 0;
  q->r_ptr = 0;
  EXIT_CRITICAL();
}

CAN_TypeDef *cans[] = {CAN1, CAN2};
CAN_HandleTypeDef *can_handles[] = {&hcan1, &hcan2};

#define CANIF_FROM_CAN_NUM(num) (cans[num])
#define CANHANDLE_FROM_CAN_NUM(num) (can_handles[num])
#define CAN_NUM_FROM_CANHANDLE(h) ((h) == &hcan1)? 0 : 1

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM6_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

// Toyota Checksum algorithm
inline uint8_t toyota_checksum(int addr, uint8_t *dat, int len){
  int cksum = 0;
  for(int ii = 0; ii < (len - 1); ii++){
    cksum = (cksum + dat[ii]);
  }
  cksum += len;
  cksum += ((addr >> 8U) & 0xFF); // idh
  cksum += ((addr) & 0xFF); // idl
  return cksum & 0xFF;
}

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void process_can(uint8_t can_number, bool tx_complete)
{
  uint32_t TxMailbox;
  CANMessage to_send;
  CAN_TxHeaderTypeDef TxHeader;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.TransmitGlobalTime = DISABLE;
  TxHeader.ExtId = 0x001;

  CAN_HandleTypeDef *handle = CANHANDLE_FROM_CAN_NUM(can_number);

  ENTER_CRITICAL();
  if (tx_complete)
  {
    can_txd_cnt ++;
  }

  while (HAL_CAN_GetTxMailboxesFreeLevel(handle))
  {
    if (!can_pop(can_queues[can_number], &to_send))
      break;

    TxHeader.StdId = to_send.Id;
    TxHeader.DLC = to_send.Size;

    if (HAL_CAN_AddTxMessage(handle, &TxHeader, to_send.Data, &TxMailbox) != HAL_OK)
    {
      /* Transmission request Error */
      Error_Handler();
    }

    can_tx_cnt ++;
  }
  EXIT_CRITICAL();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  (void)htim;
  ENTER_CRITICAL();

  if (!(features & F_FORCE_PASSTHRU))
  {
    // running for 2 minutes
    if (crash_state == CRASH_STATE_PENDING && HAL_GetTick() > 120 * 1000)
    {
      reset_crash_state();
#ifndef DEBUG
      if (can_rx_cnt == 0 || can_txd_cnt == 0)
      {
        enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
        NVIC_SystemReset();
      }
#endif
    }
  }

  // Check which version of the timer triggered this callback and toggle LED
  // called at 100Hz/20 millis
  if (acc_control_timeout < MAX_ACC_CONTROL_TIMEOUT)
  {
    acc_control_timeout += 10U;
  }

  if (pre_collision_timeout < MAX_AEB_CONTROL_TIMEOUT)
  {
    pre_collision_timeout += 10U;
  }

  if (pre_collision_2_timeout < MAX_AEB_CONTROL_TIMEOUT)
  {
    pre_collision_2_timeout += 10U;
  }

  if (aeb_timeout < MAX_AEB_TIMEOUT)
  {
    aeb_timeout += 10U;
  }

  if (crash_state != CRASH_STATE_PASSTHRU)
  {
    // 2Hz
    if (++ status_tick_count >= 50)
    {
      status_tick_count = 0;

      // status
      CANMessage to_fwd;
      to_fwd.Size = CAN_FILTER_SIZE;
      to_fwd.Id = CAN_FILTER_STATE;

      /*
      BO_ 673 CAN_FILTER_STATE: 8 XXX
      SG_ CAN2_STATUS : 0|3@1+ (1,0) [0|7] "" XXX
      SG_ CAN1_STATUS : 3|3@1+ (1,0) [0|7] "" XXX
      SG_ CRASH_STATE : 7|2@0+ (1,0) [0|3] "" XXX
      SG_ CAN_RX_ERR : 15|8@0+ (1,0) [0|255] "" XXX
      SG_ CAN_SEND_ERR : 23|8@0+ (1,0) [0|255] "" XXX
      SG_ CAN_ERR : 31|8@0+ (1,0) [0|255] "" XXX
      SG_ CAN_OVERFLOW : 39|8@0+ (1,0) [0|255] "" XXX
      SG_ UPTIME_SECONDS : 47|16@0+ (1,0) [0|65535] "" XXX
      SG_ ACC_CONTROL_TIMEOUT : 60|5@0+ (1,0) [0|255] "" XXX
      SG_ AEB_TIMEOUTS : 61|1@0+ (1,0) [0|1] "" XXX
      SG_ PRE_COLLI_TIMEOUTS : 62|1@0+ (1,0) [0|1] "" XXX
      SG_ PRE_COLLI_2_TIMEOUTS : 63|1@0+ (1,0) [0|1] "" XXX
      */

      // crash state
      to_fwd.Data[0] = crash_state << 6;

      // reset, ready, listening, sleep pending, sleep active, error
      // 3 and above treated as error
      HAL_CAN_StateTypeDef status;

      // can1 status
      status = HAL_CAN_GetState(&hcan1);
      to_fwd.Data[0] |= status << 3;

      // can2 status
      status = HAL_CAN_GetState(&hcan2);
      to_fwd.Data[0] |= status;

      to_fwd.Data[1] = can_rx_errs & 0xFF;
      to_fwd.Data[2] = can_send_errs & 0xFF;
      to_fwd.Data[3] = can_err_cnt & 0xFF;
      to_fwd.Data[4] = can_overflow_cnt & 0xFF;

      // uptime, 18 hours at most
      uint16_t uptime = HAL_GetTick() / 1000;
      to_fwd.Data[5] = (uptime & 0xFF00) >> 8;
      to_fwd.Data[6] = (uptime & 0xFF);

      // acc_control_timeout / 10, max 30
      to_fwd.Data[7] = acc_control_timeout / 10;

      // !(aeb_timeout < MAX_AEB_TIMEOUT)
      if (!(aeb_timeout < MAX_AEB_TIMEOUT))
      {
        to_fwd.Data[7] |= (1 << 5);
      }

      // !(pre_collision_timeout < MAX_AEB_CONTROL_TIMEOUT)
      if (!(pre_collision_timeout < MAX_AEB_CONTROL_TIMEOUT))
      {
        to_fwd.Data[7] |= (1 << 6);
      }

      // !(pre_collision_2_timeout < MAX_AEB_CONTROL_TIMEOUT)
      if (!(pre_collision_2_timeout < MAX_AEB_CONTROL_TIMEOUT))
      {
        to_fwd.Data[7] |= (1 << 7);
      }

      can_send_errs += can_push(can_queues[0], &to_fwd) ? 0U : 1U;

      process_can(0, false);
    }
  }

  EXIT_CRITICAL();
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
  process_can(CAN_NUM_FROM_CANHANDLE(hcan), true);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
  process_can(CAN_NUM_FROM_CANHANDLE(hcan), true);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
  process_can(CAN_NUM_FROM_CANHANDLE(hcan), true);
}

void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan)
{
  process_can(CAN_NUM_FROM_CANHANDLE(hcan), false);
}

void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan)
{
  process_can(CAN_NUM_FROM_CANHANDLE(hcan), false);
}

void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan)
{
  process_can(CAN_NUM_FROM_CANHANDLE(hcan), false);
}

void HAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan)
{
  (void)hcan;
}

void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan)
{
  (void)hcan;
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  ENTER_CRITICAL();
  can_err_cnt ++;
  HAL_CAN_ResetError(hcan);
  EXIT_CRITICAL();
}

void load_features()
{
  uint16_t *p = (uint16_t*)(CONFIG_PAGE_ADDRESS + FLASH_PAGE_SIZE - sizeof(uint16_t));
  // find last init u16
  while (*p == 0xFFFF && p > (uint16_t*)CONFIG_PAGE_ADDRESS)
  {
    p --;
  }

  if (*p == 0xFFFF)
  {
    features = (F_ACC_CONTROL | F_ACC_INIT_MAGIC | F_ACC_SPEED_LOCKOUT | F_LOW_SPEED_LEAD);
  }
  else
  {
    features = *p;
  }
}

void save_features()
{
  uint16_t *p = (uint16_t*)CONFIG_PAGE_ADDRESS;
  // find first un-init u16
  while (*p != 0xFFFF && p < (uint16_t*)(CONFIG_PAGE_ADDRESS + FLASH_PAGE_SIZE))
  {
    p ++;
  }

  HAL_FLASH_Unlock();
  if (*p != 0xFFFF)
  {
    // erase
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = CONFIG_PAGE_ADDRESS;
    erase.NbPages = 1;
    uint32_t PageError;
    if (HAL_FLASHEx_Erase(&erase, &PageError) != HAL_OK)
    {
        Error_Handler();
    }
    while (FLASH->SR & FLASH_SR_BSY);
    p = (uint16_t*)CONFIG_PAGE_ADDRESS;
  }

  HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)p, features);
  HAL_FLASH_Lock();
}

void isotp_tx()
{
  isotp_tx_ptr = isotp_tx_buf;
  isotp_tx_index = 1;
  if (isotp_tx_remain <= 0 || isotp_tx_remain > sizeof(isotp_tx_buf) - 0x10)
    return;

  if (isotp_tx_remain <= 7)
  {
    // 0: Single frame (SF)
    // The single frame transferred contains the complete payload of up to 7 bytes
    // (normal addressing) or 6 bytes (extended addressing)

    CANMessage tx;
    tx.Size = 8;
    tx.Id = CAN_FILTER_ISOTP_TX;
    tx.Data[0] = 0x00 | isotp_tx_remain;
    memcpy(tx.Data + 1, isotp_tx_ptr, isotp_tx_remain);
    can_send_errs += can_push(can_queues[0], &tx) ? 0U: 1U;
  }
  else
  {
    // 1: First frame (FF)
    // The first frame of a longer multi-frame message packet, used when more
    // than 6/7 bytes of data segmented must be communicated. The first frame
    // contains the length of the full packet, along with the initial data.

    CANMessage tx;
    tx.Size = 8;
    tx.Id = CAN_FILTER_ISOTP_TX;
    // type | high 4bits of length
    tx.Data[0] = 0x10 | (isotp_tx_remain >> 8);
    // low 8 bits of length
    tx.Data[1] = isotp_tx_remain & 0xFF;
    memcpy(tx.Data + 2, isotp_tx_ptr, 6);
    isotp_tx_remain -= 6;
    isotp_tx_ptr += 6;
    can_send_errs += can_push(can_queues[0], &tx) ? 0U: 1U;
  }
}

int isotp_on_message(uint8_t* rx_buf, int len, uint8_t* tx_buf, int tx_size)
{
  uint8_t type = rx_buf[0];
  switch (type)
  {
    // log
    case 0x00:
      {
        int len = 0;
        ring_buffer* q = &ring_buffer_debug;
        ENTER_CRITICAL();
        while (q->w_ptr_tx != q->r_ptr_tx && len < tx_size)
        {
          tx_buf[len++] = q->elems_tx[q->r_ptr_tx];
          q->r_ptr_tx = (q->r_ptr_tx + 1U) % q->tx_fifo_size;
        }
        EXIT_CRITICAL();
        return len;
      }
      break;

    // signature
    case 0x01:
      {
        char * code = (char*)_app_start;
        int code_len = _app_start[0];
        (void)memcpy(tx_buf, &code[code_len], 128);
        return 128;
      }
      break;

    // state
    case 0x02:
      {
        HAL_CAN_StateTypeDef state;
        uint8_t* p = tx_buf;
        uint16_t uptime = HAL_GetTick() / 1000;
        (void)memcpy(p, &uptime, sizeof(uptime)); p += sizeof(uptime);
        (void)memcpy(p, &crash_state, sizeof(crash_state)); p += sizeof(crash_state);
        state = HAL_CAN_GetState(&hcan1);
        (void)memcpy(p, &state, sizeof(state)); p += sizeof(state);
        state = HAL_CAN_GetState(&hcan2);
        (void)memcpy(p, &state, sizeof(state)); p += sizeof(state);
        (void)memcpy(p, &acc_control_timeout, sizeof(acc_control_timeout)); p += sizeof(acc_control_timeout);
        (void)memcpy(p, &aeb_timeout, sizeof(aeb_timeout)); p += sizeof(aeb_timeout);
        (void)memcpy(p, &pre_collision_timeout, sizeof(pre_collision_timeout)); p += sizeof(pre_collision_timeout);
        (void)memcpy(p, &pre_collision_2_timeout, sizeof(pre_collision_2_timeout)); p += sizeof(pre_collision_2_timeout);
        (void)memcpy(p, gitversion, sizeof(gitversion)); p += sizeof(gitversion);
        (void)memcpy(p, &features, sizeof(features)); p += sizeof(features);
        (void)memcpy(p, &can_rx_errs, sizeof(uint32_t)); p += sizeof(uint32_t);
        (void)memcpy(p, &can_send_errs, sizeof(uint32_t)); p += sizeof(uint32_t);
        (void)memcpy(p, &can_rx_cnt, sizeof(uint32_t)); p += sizeof(uint32_t);
        (void)memcpy(p, &can_tx_cnt, sizeof(uint32_t)); p += sizeof(uint32_t);
        (void)memcpy(p, &can_txd_cnt, sizeof(uint32_t)); p += sizeof(uint32_t);
        (void)memcpy(p, &can_err_cnt, sizeof(uint32_t)); p += sizeof(uint32_t);
        (void)memcpy(p, &can_overflow_cnt, sizeof(uint32_t)); p += sizeof(uint32_t);
        return p - tx_buf;
      }
      break;

    default:
      tx_buf[0] = 0xFF;
      return 1;
  }
}

void can_rx(uint8_t can_number, uint32_t fifo)
{
  CAN_HandleTypeDef* handle = CANHANDLE_FROM_CAN_NUM(can_number);
  CAN_RxHeaderTypeDef   RxHeader;
  uint8_t               RxData[8];
  uint8_t fwd_can = (can_number == 0) ? 1 : 0;
  CANMessage to_fwd;
  uint32_t free_level;

  while (true)
  {
    ENTER_CRITICAL();
    free_level = HAL_CAN_GetRxFifoFillLevel(handle, fifo);
    if (free_level > 0)
    {
      /* Get RX message */
      if (HAL_CAN_GetRxMessage(handle, fifo, &RxHeader, RxData) != HAL_OK)
      {
        /* Reception Error */
        can_rx_errs ++;
        Error_Handler();
      }

      can_rx_cnt ++;
    }
    EXIT_CRITICAL();

    if (free_level <= 0)
    {
      return;
    }

    // internal magic msg
    if (RxHeader.StdId == CAN_FILTER_INPUT_MAGIC && RxHeader.DLC == CAN_FILTER_SIZE && can_number == 0)
    {
      // magic
      if (memcmp(RxData, "\xce\xfa\xad\xde", 4) == 0)
      {
        // bootloader
        if (memcmp(RxData + 4, "\x1e\x0b\xb0\x02", 4) == 0)
        {
          reset_crash_state();
          enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
          NVIC_SystemReset();
        }
        // softloader
        else if (memcmp(RxData + 4, "\x1e\x0b\xb0\x0a", 4) == 0)
        {
          reset_crash_state();
          enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
          NVIC_SystemReset();
        }
        // reset
        else if (memcmp(RxData + 4, "\x1e\x0b\xb0\x01", 4) == 0)
        {
          reset_crash_state();
          NVIC_SystemReset();
        }
        // feature
        else if (RxData[4] == 0x1F)
        {
          features = RxData[6] << 8 | RxData[7];
          if (RxData[5] & 0x1)
          {
            save_features();
          }
        }
      }
      return;
    }

    // skip on failsafe
    if (crash_state != CRASH_STATE_PASSTHRU)
    {
      if (can_number == 0)
      {
        // (OVERWRITE) ACC control msg on can 0, EON is sending
        if (RxHeader.StdId == CAN_FILTER_ACC_CONTROL && RxHeader.DLC == 8)
        {
          memcpy(to_fwd.Data, RxData, 8);
          // evict the oldest if failed to push
          if (!can_push(&can_acc_control_q, &to_fwd))
          {
            CANMessage dummy;
            can_pop(&can_acc_control_q, &dummy);
            can_push(&can_acc_control_q, &to_fwd);
          }

          acc_control_timeout = 0;

          // to be translated, no forward
          RxHeader.DLC = 0;
        }
        // (OVERWRITE) ACC control msg on can 0, EON is sending
        else if (RxHeader.StdId == 0x343 && RxHeader.DLC == 8)
        {
          acc_control_timeout = 0;

          // no forward
          RxHeader.DLC = 0;
        }
        // (OVERWRITE) PRE COLLISION 2
        else if (RxHeader.StdId == CAN_FILTER_PRE_COLLISION_2 && RxHeader.DLC == 8)
        {
          memcpy(pre_collision_2_data, RxData, 7);
          pre_collision_2_present = true;
          pre_collision_2_timeout = 0;

          // no forward
          RxHeader.DLC = 0;
        }
        // (OVERWRITE) PRE COLLISION 2
        else if (RxHeader.StdId == 0x344 && RxHeader.DLC == 8)
        {
          pre_collision_2_timeout = 0;

          // no forward
          RxHeader.DLC = 0;
        }
        // (OVERWRITE) PRE COLLISION
        else if (RxHeader.StdId == 0x283 && RxHeader.DLC == 7)
        {
          pre_collision_timeout = 0;

          // no forward
          RxHeader.DLC = 0;
        }
        // PCM_CRUISE_2: LOW_SPEED_LOCKOUT 3
        else if (RxHeader.StdId == 0x1D3 && RxHeader.DLC == 8)
        {
          low_speed_lockout = (RxData[1] >> 5) & 0x3;
        }
        // SPEED
        else if (RxHeader.StdId == 0xB4 && RxHeader.DLC == 8)
        {
          uint16_t speed = (RxData[5] << 8) | RxData[6];
          vehicle_speed = (float)speed * 0.01;
        }
        // PCM CRUISE
        else if (RxHeader.StdId == 0x1D2 && RxHeader.DLC == 8)
        {
          cruise_active = ((RxData[0] & 0x20) != 0) ? true : false;
        }
        // ISOTP_RX
        else if (RxHeader.StdId == CAN_FILTER_ISOTP_RX && RxHeader.DLC == 8)
        {
          uint8_t type = RxData[0] & 0xF0;
          // 3: Flow control frame (FC)
          // the response from the receiver, acknowledging a First-frame segment.
          // It lays down the parameters for the transmission of further consecutive frames.
          if (type == (0x3 << 4))
          {
            // RxData[0] & 0xF : FC flag (0,1,2)
            // RxData[1]       : Block size
            // RxData[2]       : ST
            while (isotp_tx_remain > 0)
            {
              // 2: Consecutive frame (CF)
              // A frame containing subsequent data for a multi-frame packet
              CANMessage tx;
              tx.Size = 8;
              tx.Id = CAN_FILTER_ISOTP_TX;
              tx.Data[0] = 0x20 | (isotp_tx_index & 0xF);
              memcpy(tx.Data + 1, isotp_tx_ptr, 7);
              isotp_tx_remain -= 7;
              isotp_tx_ptr += 7;
              isotp_tx_index ++;
              can_send_errs += can_push(can_queues[0], &tx) ? 0U: 1U;
            }
          }
          // 2: Consecutive frame (CF)
          // A frame containing subsequent data for a multi-frame packet
          else if (type == (0x2 << 4))
          {
            if (isotp_rx_remain > 0)
            {
              memcpy(isotp_rx_ptr, RxData + 1, 7);
              isotp_rx_ptr += 7;
              isotp_rx_remain -= 7;

              // full message recevied
              if (isotp_rx_remain <= 0)
              {
                // call on message
                int len = isotp_rx_ptr - isotp_rx_buf + isotp_rx_remain;
                isotp_tx_remain = isotp_on_message(isotp_rx_buf, len, isotp_tx_buf, sizeof(isotp_tx_buf) - 0x10);
                isotp_tx();
              }
            }
          }
          // 1: First frame (FF)
          // The first frame of a longer multi-frame message packet, used when more
          // than 6/7 bytes of data segmented must be communicated. The first frame
          // contains the length of the full packet, along with the initial data.
          else if (type == (0x1 << 4))
          {
            int len = (RxData[0] & 0xF) << 8 | RxData[1];
            isotp_rx_ptr = isotp_rx_buf;
            memcpy(isotp_rx_ptr, RxData + 2, 6);
            // bounds check
            if (len < sizeof(isotp_rx_buf) - 0x10)
            {
              isotp_rx_ptr += 6;
              isotp_rx_remain = len - 6;
            }

            CANMessage tx;
            tx.Size = 8;
            tx.Id = CAN_FILTER_ISOTP_TX;
            // 3: Flow control frame (FC)
            RxData[0] = (0x3 << 4);
            memset(RxData + 1, 0, 7);
            can_send_errs += can_push(can_queues[0], &tx) ? 0U: 1U;
          }
          // 0: Single frame (SF)
          // The single frame transferred contains the complete payload of up to 7 bytes
          // (normal addressing) or 6 bytes (extended addressing)
          else if (type == (0x0 << 4))
          {
            int len = RxData[0] & 0xF;
            // call on message
            isotp_tx_remain = isotp_on_message(RxData + 1, len, isotp_tx_buf, sizeof(isotp_tx_buf) - 0x10);
            isotp_tx();
          }

          // no forward
          RxHeader.DLC = 0;
        }
      }
      else
      // can 1
      {
        // PRE COLLISION 2, 20Hz
        if (RxHeader.StdId == 0x344 && RxHeader.DLC == 8)
        {
          //0,0x344,0x0000010000000050,8
          if (RxData[7] != 0x50)
          {
            aeb_timeout = 0;
          }

          // safe to overwrite?
          if (!(aeb_timeout < MAX_AEB_TIMEOUT) && pre_collision_2_timeout < MAX_AEB_CONTROL_TIMEOUT)
          {
            // drop msg is fixed 0x344,0x0000010000000050,8, do not have to copy to can 0

            // miss msg? wait for timeout
            if (!pre_collision_2_present)
            {
                return; // drop
            }

            // overwrite (with content from EON)
            memcpy(RxData, pre_collision_2_data, 7);
            RxData[7] = toyota_checksum(0x344, RxData, 8);
            pre_collision_2_present = false;
          }
        }
        // PRE_COLLISION, 33.33Hz, PRECOLLISION_ACTIVE: RxData[5] & 0x2, warning on dash
        else if (RxHeader.StdId == 0x283 && RxHeader.DLC == 7)
        {
          //0,0x283,0x0000000000008c,7
          if (RxData[6] != 0x8C)
          {
            aeb_timeout = 0;
          }

          // overwrite (drop stock, allow EON)
          if (!(aeb_timeout < MAX_AEB_TIMEOUT) && pre_collision_timeout < MAX_AEB_CONTROL_TIMEOUT)
          {
            // drop msg is fixed 0x344,0x0000010000000050,8, do not have to copy to can 0

            return;
          }
        }
        // ACC CONTROL, 33.33Hz
        else if ((features & F_ACC_CONTROL) && RxHeader.StdId == 0x343 && RxHeader.DLC == 8)
        {
          // copy to CAN 0 with a different id
          to_fwd.Size = 8;
          to_fwd.Id = CAN_FILTER_ACC_CONTROL_COPY;
          memcpy(to_fwd.Data, RxData, 7);
          to_fwd.Data[7] = toyota_checksum(CAN_FILTER_ACC_CONTROL_COPY, to_fwd.Data, 8);
          can_send_errs += can_push(can_queues[fwd_can], &to_fwd) ? 0U: 1U;

          if (!(aeb_timeout < MAX_AEB_TIMEOUT))
          {
            // EON is sending, ignore this msg
            if (acc_control_timeout < MAX_ACC_CONTROL_TIMEOUT)
            {
              // load ACC control (overwrite) msg
              if (!can_pop(&can_acc_control_q, &to_fwd))
              {
                // send acc_control_copy before return
                process_can(fwd_can, false);
                return; // drop
              }

              // enforce error display on dash
              // ACC_MALFUNCTION
              if ((RxData[2] & 0x4) != 0)
              {
                  to_fwd.Data[2] |= 0x4;
              }

              // RADAR_DIRTY
              if ((RxData[2] & 0x8) != 0)
              {
                  to_fwd.Data[2] |= 0x8;
              }

              // ACC_CUT_IN
              if ((RxData[3] & 0x2) != 0)
              {
                  to_fwd.Data[3] |= 0x2;
              }

              // overwrite (with content from EON)
              memcpy(RxData, to_fwd.Data, 7);
              RxData[7] = toyota_checksum(0x343, RxData, 8);
            }
            else
            {
              // initializing, inject fake msg
              if ((features & F_ACC_INIT_MAGIC) && low_speed_lockout == 3)
              {
                // mimic CH-R behavir
                uint8_t acc_control[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
                if (HAL_GetTick() < 800)
                {
                  // acc type: 0, smc: 1
                  acc_control[2] = 0x01;
                }
                else if (HAL_GetTick() > 1800)
                {
                  // acc type: 1, smc: 3
                  acc_control[2] = 0x43;
                }
                else
                {
                  // acc type: 1, smc: 1
                  acc_control[2] = 0x41;
                }

                memcpy(RxData, acc_control, sizeof(acc_control));
                RxData[7] = toyota_checksum(0x343, RxData, 8);
              }
              else
              {
                // use stock msg, but update acc type if acc type is 2
                if ((RxData[2] & 0xC0) == 0x80)
                {
                  // clear acc_type 0xc0
                  RxData[2] &= 0x3F;
                  // add acc_type 0x40 and allow_long_press 0x01
                  RxData[2] |= 0x41;

                  // 45 on dash
                  if (vehicle_speed < 41.5)
                  {
                    // engage at 30kph, disengage at 25kph
                    // disable lead car to disengage, or disable engagement
                    if ((features & F_ACC_SPEED_LOCKOUT) &&
                        ((cruise_active && vehicle_speed < 21.0) || ((!cruise_active) && vehicle_speed < 26.0)))
                    {
                      // no lead car, clear mini_car 0x20
                      RxData[2] &= 0xDF;
                      // lead standstill to 0, clear lead_standstill 0x20
                      RxData[3] &= 0xDF;
                    }
                    // fake moving lead
                    else if (features & F_LOW_SPEED_LEAD)
                    {
                      // no lead car
                      if ((RxData[2] & 0x20) == 0)
                      {
                        // lead car
                        RxData[2] |= 0x20;
                        // lead standstill to 0, clear lead_standstill 0x20
                        RxData[3] &= 0xDF;
                      }
                    }
                  }

                  // update checksum
                  RxData[7] = toyota_checksum(0x343, RxData, 8);
                }
              }
            }
          }
          // clear acc_control queue while aeb running
          else
          {
            can_clear(&can_acc_control_q);
          }
        }

        // 0x33e, 5Hz, 7fff00c0000000 on CH-R at boot

        // 0x365 DSU_CRUISE, 5Hz, LEAD_DISTANCE: RxData[4]

        // 0x366, 5Hz
        // BO_ 870 DS11D71: 7 DS1
        //  SG_ XREQALM : 7|1@0+ (1,0) [0|0] "" Vector__XXX
        //  SG_ XREQABK : 6|1@0+ (1,0) [0|0] "" Vector__XXX
        //  SG_ TGT_DIST : 5|14@0+ (0.01,0) [0|0] "m" Vector__XXX
        //  SG_ XREQPBA : 23|1@0+ (1,0) [0|0] "" Vector__XXX
        //  SG_ XREQFPB : 22|1@0+ (1,0) [0|0] "" Vector__XXX
        //  SG_ XREQPB : 21|1@0+ (1,0) [0|0] "" Vector__XXX
        //  SG_ XREQEXT : 20|1@0+ (1,0) [0|0] "" Vector__XXX
        //  SG_ XREQPSB : 19|1@0+ (1,0) [0|0] "" Vector__XXX
        //  SG_ TGT_VGAP : 18|11@0+ (0.025,-51.175) [0|0] "m/s" Vector__XXX
        //  SG_ PCSDISP : 39|4@0+ (1,0) [0|0] "" Vector__XXX
        //  SG_ XPCSRDY : 35|1@0+ (1,0) [0|0] "" Vector__XXX
        //  SG_ TGT_NUMB : 34|3@0+ (1,1) [0|0] "" Vector__XXX
        //  SG_ TGT_POSX : 47|8@0- (0.04,0) [0|0] "m" Vector__XXX

        // 0x411 ACC_HUD, 1Hz, FCW: RxData[0] & 0x10
        // BO_ 1041 DS12F02: 8 DS1
        //  SG_ PCSINDI : 7|2@0+ (1,0) [0|0] "" FCM
        //  SG_ PCSWM : 5|2@0+ (1,0) [0|0] "" FCM
        //  SG_ PCSFCT : 3|1@0+ (1,0) [0|0] "" FCM
        //  SG_ PCSTUCT : 2|1@0+ (1,0) [0|0] "" FCM
        //  SG_ DS1LCCK : 1|2@0+ (1,0) [0|0] "" FCM
        //  SG_ PBTUCT : 14|1@0+ (1,0) [0|0] "" FCM
        //  SG_ PCSEXIST : 13|1@0+ (1,0) [0|0] "" Vector__XXX
        //  SG_ PCSWDUCT : 11|1@0+ (1,0) [0|0] "" Vector__XXX
        //  SG_ PCSWD : 9|2@0+ (1,0) [0|0] "" Vector__XXX
        //  SG_ PCSDW : 39|3@0+ (1,0) [0|0] "" Vector__XXX
        //  SG_ PCSDSRF : 36|1@0+ (1,0) [0|0] "" Vector__XXX
        //  SG_ PCSTEMP : 35|1@0+ (1,0) [0|0] "" Vector__XXX
        //  SG_ PCSDUST : 34|1@0+ (1,0) [0|0] "" Vector__XXX
        //  SG_ PCSLCCK : 33|2@0+ (1,0) [0|0] "" Vector__XXX
        //  SG_ PCSPEDW : 47|3@0+ (1,0) [0|0] "" Vector__XXX
        //  SG_ PCSPVSN : 44|2@0+ (1,0) [0|0] "" Vector__XXX
        //  SG_ PCTEMP2 : 42|1@0+ (1,0) [0|0] "" Vector__XXX
        //  SG_ PCSDUST2 : 41|1@0+ (1,0) [0|0] "" Vector__XXX
        //  SG_ PCSOFFS : 40|1@0+ (1,0) [0|0] "" Vector__XXX
        //  SG_ PCSWDS : 55|2@0+ (1,0) [0|0] "" Vector__XXX
        //  SG_ FRDADJ : 53|3@0+ (1,0) [0|0] "" Vector__XXX

        // 0x4ff, 0.77Hz, 0:4ff 3f00000000000000
        // always 3f00000000000000, even on CH-R
        // BO_ 1279 FRD1N01: 8 FRD
        //  SG_ FRDNID : 7|8@0+ (1,0) [0|0] "" CGW
        //  SG_ FRDSNG : 15|1@0+ (1,0) [0|0] "" CGW
        //  SG_ FRDSPF : 23|16@0+ (1,0) [0|0] "" CGW
        //  SG_ FRDREV : 39|32@0+ (1,0) [0|0] "" CGW

      } // can 1

    } // if (crash_state != CRASH_STATE_PASSTHRU)

    if (RxHeader.DLC > 0)
    {
      to_fwd.Size = RxHeader.DLC;
      to_fwd.Id = RxHeader.StdId;
      memcpy(to_fwd.Data, RxData, to_fwd.Size);
      can_send_errs += can_push(can_queues[fwd_can], &to_fwd) ? 0U: 1U;
    }
  } // while (true)

  process_can(fwd_can, false);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
  can_rx(CAN_NUM_FROM_CANHANDLE(CanHandle), CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
  can_rx(CAN_NUM_FROM_CANHANDLE(CanHandle), CAN_RX_FIFO1);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* c init, which is disabled in startup file */
  can_rx_errs = 0;
  can_send_errs = 0;
  can_rx_cnt = 0;
  can_tx_cnt = 0;
  can_txd_cnt = 0;
  can_err_cnt = 0;
  can_overflow_cnt = 0;

  low_speed_lockout = 3;
  vehicle_speed = 0;
  cruise_active = false;

  acc_control_timeout = MAX_ACC_CONTROL_TIMEOUT;
  pre_collision_timeout = MAX_AEB_CONTROL_TIMEOUT;
  pre_collision_2_timeout = MAX_AEB_CONTROL_TIMEOUT;
  pre_collision_2_present = false;

  aeb_timeout = MAX_AEB_TIMEOUT;

  status_tick_count = 0;

  debug_ring_ready = false;

  // isotp
  isotp_rx_ptr = NULL;
  isotp_rx_remain = 0;
  isotp_tx_ptr = NULL;
  isotp_tx_remain = 0;
  isotp_tx_index = 0;

  load_features();

  if (features & F_FORCE_PASSTHRU)
  {
    crash_state = CRASH_STATE_PASSTHRU;
  }
  else
  {
    update_crash_state();
  }

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_DeInit();

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM6_Init();

#ifndef DEBUG
  MX_IWDG_Init();
#endif
  /* USER CODE BEGIN 2 */

  /* Configure the CAN Filter */
  CAN_FilterTypeDef sFilterConfig1;
  sFilterConfig1.FilterBank = 0;
  sFilterConfig1.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig1.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig1.FilterIdHigh = 0x0000;
  sFilterConfig1.FilterIdLow = 0x0000;
  sFilterConfig1.FilterMaskIdHigh = 0x0000;
  sFilterConfig1.FilterMaskIdLow = 0x0000;
  sFilterConfig1.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig1.FilterActivation = ENABLE;
  sFilterConfig1.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig1) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  /* Start the CAN peripheral */
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  /* Activate CAN RX notification */
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING |
              CAN_IT_RX_FIFO1_MSG_PENDING |
              CAN_IT_TX_MAILBOX_EMPTY |
              CAN_IT_ERROR |
              CAN_IT_LAST_ERROR_CODE |
              CAN_IT_ERROR_PASSIVE |
              CAN_IT_ERROR_WARNING) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }

  clear_ring_buffer(&ring_buffer_debug);
  debug_ring_ready = true;

  dbg_puts("can filter starts!\n");

  /* Configure the CAN Filter */
  CAN_FilterTypeDef sFilterConfig2;
  sFilterConfig2.FilterBank = 14;
  sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig2.FilterIdHigh = 0x0000;
  sFilterConfig2.FilterIdLow = 0x0000;
  sFilterConfig2.FilterMaskIdHigh = 0x0000;
  sFilterConfig2.FilterMaskIdLow = 0x0000;
  sFilterConfig2.FilterFIFOAssignment = CAN_RX_FIFO1;
  sFilterConfig2.FilterActivation = ENABLE;
  sFilterConfig2.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig2) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  /* Start the CAN peripheral */
  if (HAL_CAN_Start(&hcan2) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  /* Activate CAN RX notification */
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING |
              CAN_IT_RX_FIFO1_MSG_PENDING |
              CAN_IT_TX_MAILBOX_EMPTY |
              CAN_IT_ERROR |
              CAN_IT_LAST_ERROR_CODE |
              CAN_IT_ERROR_PASSIVE |
              CAN_IT_ERROR_WARNING) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }

  /* Configure Transmission process */

  /* USER CODE END 2 */
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }

  enable_interrupts();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (true)
  {
    /* USER CODE END WHILE */

#ifndef DEBUG
    // feed the dog, at least every 10 milliseconds
    HAL_IWDG_Refresh(&hiwdg);
#endif


    process_can(0, false);

    process_can(1, false);

    HAL_Delay(5);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_LSI_Enable();

   /* Wait till LSI is ready */
  while(LL_RCC_LSI_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_PLL2_DIV_5, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_ConfigDomain_PLL2(LL_RCC_HSE_PREDIV2_DIV_5, LL_RCC_PLL2_MUL_8);
  LL_RCC_PLL2_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL2_IsReady() != 1)
  {

  }
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(72000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeTriggeredMode = ENABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;

#if 0
  hcan1.Init.Prescaler = 4;
  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
#else
  hcan1.Init.Prescaler = 9;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
#endif

  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeTriggeredMode = ENABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;

#if 0
  hcan2.Init.Prescaler = 4;
  hcan2.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
#else
  hcan2.Init.Prescaler = 9;
  hcan2.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
#endif

  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  // reset every 10 ms
  // 72000000/256/2812 = 100 Hz
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 2812;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7199;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();

  // reset
  NVIC_SystemReset();

/*
  while (true)
  {
  }
*/
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
