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

#include "early_init.h"

#include "stm32f1xx_it.c.h"
#include "build/gitversion.h"

#define ENABLE_ACC_INIT_MAGIC      true

#define CAN_FILTER_SIZE     8
#define CAN_FILTER_INPUT    0x2A0U
#define CAN_FILTER_OUTPUT   0x2A1U
#define CAN_FILTER_MUX      (CAN_FILTER_OUTPUT + 0)
#define CAN_FILTER_ERR      (CAN_FILTER_OUTPUT + 1)
#define CAN_FILTER_LOG      (CAN_FILTER_OUTPUT + 2)

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

uint32_t can_rx_errs = 0;
uint32_t can_send_errs = 0;

uint32_t can_rx_cnt = 0;
uint32_t can_tx_cnt = 0;
uint32_t can_txd_cnt = 0;
uint32_t can_err_cnt = 0;
uint32_t can_csum_err_cnt = 0;
uint32_t can_overflow_cnt = 0;

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
TIM_HandleTypeDef htim6;

IWDG_HandleTypeDef hiwdg;

uint8_t low_speed_lockout = 3;
bool stock_aeb_active = false;
uint16_t car_speed = 0;
float cruise_active = 0.0f;

// max 1000ms
#define MAX_ACC_CONTROL_TIMEOUT 1000

uint32_t acc_control_timeout = MAX_ACC_CONTROL_TIMEOUT;
uint8_t status_tick_count = 0;
uint8_t error_tick_count = 0;
bool debug_ring_ready = false;

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

// ******************************** UART buffers ********************************
UART_BUFFER(debug, DEBUG_FIFO_SIZE)

void tx_ring_buffer(ring_buffer *q)
{
  if (q->w_ptr_tx == q->r_ptr_tx)
  {
    return;
  }

  CAN_TxHeaderTypeDef TxHeader;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.TransmitGlobalTime = DISABLE;
  TxHeader.ExtId = 0x001;
  TxHeader.StdId = CAN_FILTER_LOG;
  TxHeader.DLC = 0;
  uint8_t Data[8];

  // log
  uint32_t TxMailbox;

  ENTER_CRITICAL();
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) &&
          q->w_ptr_tx != q->r_ptr_tx &&
          TxHeader.DLC < 8)
  {
    Data[TxHeader.DLC++] = q->elems_tx[q->r_ptr_tx];
    q->r_ptr_tx = (q->r_ptr_tx + 1U) % q->tx_fifo_size;
  }

  if (TxHeader.DLC > 0)
  {
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Data, &TxMailbox) != HAL_OK)
    {
      /* Transmission request Error */
      Error_Handler();
    }

    can_tx_cnt ++;
  }
  EXIT_CRITICAL();
}

void tx_debug_ring()
{
    tx_ring_buffer(&ring_buffer_debug);
}

void clear_ring_buffer(ring_buffer *q) {
  ENTER_CRITICAL();
  q->w_ptr_tx = 0;
  q->r_ptr_tx = 0;
  EXIT_CRITICAL();
}

bool putc(ring_buffer *q, char elem) {
  if (!debug_ring_ready)
  {
    return false;
  }

  bool ret = false;
  uint16_t next_w_ptr;

  ENTER_CRITICAL();
  next_w_ptr = (q->w_ptr_tx + 1U) % q->tx_fifo_size;
  if (next_w_ptr != q->r_ptr_tx) {
    q->elems_tx[q->w_ptr_tx] = elem;
    q->w_ptr_tx = next_w_ptr;
    ret = true;
  }
  EXIT_CRITICAL();
  //tx_ring_buffer(q);
  return ret;
}

void putch(const char a) {
  // do not lock the cpu
  //while (!putc(&ring_buffer_debug, a));
  putc(&ring_buffer_debug, a);
}

void puts(const char *a) {
  for (const char *in = a; *in; in++) {
    if (*in == '\n') putch('\r');
    putch(*in);
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
  puts(&str[idx + 1U]);
}

void puth(unsigned int i) {
  const char c[] = "0123456789abcdef";
  for (int pos = 28; pos != -4; pos -= 4) {
    putch(c[(i >> (unsigned int)(pos)) & 0xFU]);
  }
}

void puth2(unsigned int i) {
  const char c[] = "0123456789abcdef";
  for (int pos = 4; pos != -4; pos -= 4) {
    putch(c[(i >> (unsigned int)(pos)) & 0xFU]);
  }
}

void hexdump(const void *a, int l) {
  if (a != NULL) {
    for (int i=0; i < l; i++) {
      if ((i != 0) && ((i & 0xf) == 0)) puts("\n");
      puth2(((const unsigned char*)a)[i]);
      puts(" ");
    }
  }
  puts("\n");
}

// global CAN stats
typedef struct {
  uint32_t Id;
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
      puts("can_push failed!\n");
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
uint8_t toyota_checksum(int addr, uint8_t *dat, int len){
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
void process_can(uint8_t can_number)
{
  CAN_HandleTypeDef *handle = CANHANDLE_FROM_CAN_NUM(can_number);
  while (HAL_CAN_GetTxMailboxesFreeLevel(handle))
  {
    CANMessage to_send;
    if (!can_pop(can_queues[can_number], &to_send))
      break;

    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.TransmitGlobalTime = DISABLE;
    TxHeader.ExtId = 0x001;
    TxHeader.StdId = to_send.Id;
    TxHeader.DLC = to_send.Size;

    uint32_t TxMailbox;
    if (HAL_CAN_AddTxMessage(handle, &TxHeader, to_send.Data, &TxMailbox) != HAL_OK)
    {
      /* Transmission request Error */
      Error_Handler();
    }

    can_tx_cnt ++;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  ENTER_CRITICAL();
  (void)htim;
  // Check which version of the timer triggered this callback and toggle LED
  // called at 50Hz/20 millis
  if (acc_control_timeout < MAX_ACC_CONTROL_TIMEOUT)
  {
      acc_control_timeout += 20U;
  }

  // 1Hz
  if (++ status_tick_count >= 50)
  {
    status_tick_count = 0;

    // status
    CANMessage to_fwd;
    to_fwd.Size = CAN_FILTER_SIZE;
    to_fwd.Id = CAN_FILTER_MUX;
    uint16_t uptime = HAL_GetTick() / 1000;
    to_fwd.Data[0] = 0x00;
    // reset, ready, listening, sleep pending, sleep active, error
    HAL_CAN_StateTypeDef status = HAL_CAN_GetState(&hcan1);
    // treat 3 and plus as error
    to_fwd.Data[0] |= MIN(status, 3) << 2;
    status = HAL_CAN_GetState(&hcan2);
    to_fwd.Data[0] |= MIN(status, 3);
    memcpy(to_fwd.Data + 1, gitversion, sizeof(gitversion));
    to_fwd.Data[5] = (uptime & 0xFF00) >> 8;
    to_fwd.Data[6] = (uptime & 0xFF);
    to_fwd.Data[7] = toyota_checksum(CAN_FILTER_MUX, to_fwd.Data, 8);
    can_send_errs += can_push(can_queues[0], &to_fwd) ? 0U : 1U;

    // send
    process_can(0);
  }

  // 5Hz
  if (++ error_tick_count >= 10)
  {
    error_tick_count = 0;

    // error status
    CANMessage to_fwd;
    to_fwd.Size = CAN_FILTER_SIZE;
    to_fwd.Id = CAN_FILTER_ERR;
    #define GET_ERROR(x) (x > 0xFF) ? 0xFF : (x)
    to_fwd.Data[0] = GET_ERROR(can_rx_errs);
    to_fwd.Data[1] = GET_ERROR(can_send_errs);
    to_fwd.Data[2] = GET_ERROR(can_rx_cnt);
    to_fwd.Data[3] = GET_ERROR(can_tx_cnt);
    to_fwd.Data[4] = GET_ERROR(can_txd_cnt);
    to_fwd.Data[5] = GET_ERROR(can_err_cnt);
    to_fwd.Data[6] = GET_ERROR(can_csum_err_cnt);
    to_fwd.Data[7] = GET_ERROR(can_overflow_cnt);
    #undef GET_ERROR
    can_send_errs += can_push(can_queues[0], &to_fwd) ? 0U : 1U;

    // send
    process_can(0);
  }

  EXIT_CRITICAL();
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
  ENTER_CRITICAL();
  can_txd_cnt ++;
  process_can(CAN_NUM_FROM_CANHANDLE(hcan));
  tx_debug_ring();
  EXIT_CRITICAL();
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
  ENTER_CRITICAL();
  can_txd_cnt ++;
  process_can(CAN_NUM_FROM_CANHANDLE(hcan));
  tx_debug_ring();
  EXIT_CRITICAL();
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
  ENTER_CRITICAL();
  can_txd_cnt ++;
  process_can(CAN_NUM_FROM_CANHANDLE(hcan));
  tx_debug_ring();
  EXIT_CRITICAL();
}

void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan)
{
  ENTER_CRITICAL();
  process_can(CAN_NUM_FROM_CANHANDLE(hcan));
  tx_debug_ring();
  EXIT_CRITICAL();
}

void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan)
{
  ENTER_CRITICAL();
  process_can(CAN_NUM_FROM_CANHANDLE(hcan));
  tx_debug_ring();
  EXIT_CRITICAL();
}

void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan)
{
  ENTER_CRITICAL();
  process_can(CAN_NUM_FROM_CANHANDLE(hcan));
  tx_debug_ring();
  EXIT_CRITICAL();
}

void HAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan)
{
}

void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan)
{
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  ENTER_CRITICAL();
  can_err_cnt ++;
  HAL_CAN_ResetError(hcan);
  EXIT_CRITICAL();
}

void can_rx(uint8_t can_number)
{
  CAN_HandleTypeDef* handle = CANHANDLE_FROM_CAN_NUM(can_number);
  CAN_RxHeaderTypeDef   RxHeader;
  uint8_t               RxData[8];
  /* Get RX message */
  if (HAL_CAN_GetRxMessage(handle, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    /* Reception Error */
    can_rx_errs ++;
    Error_Handler();
  }

  can_rx_cnt ++;

  if (can_number == 0)
  {
    // internal magic msg
    if (RxHeader.StdId == CAN_FILTER_INPUT && RxHeader.DLC == CAN_FILTER_SIZE)
    {
      // magic
      if (memcmp(RxData, "\xce\xfa\xad\xde", 4) == 0)
      {
        // bootloader
        if (memcmp(RxData + 4, "\x1e\x0b\xb0\x02", 4) == 0)
        {
          enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
          NVIC_SystemReset();
        }
        // softloader
        else if (memcmp(RxData + 4, "\x1e\x0b\xb0\x0a", 4) == 0)
        {
          enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
          NVIC_SystemReset();
        }
        // reset
        else if (memcmp(RxData + 4, "\x1e\x0b\xb0\x01", 4) == 0)
        {
          NVIC_SystemReset();
        }
        // mute
        else if (memcmp(RxData + 4, "\x1e\x0b\xb0\x03", 4) == 0)
        {
          disable_interrupts();
          // reset to recover
          while (true)
          {
          }
        }
      }
      return;
    }
    // PCM_CRUISE_2: LOW_SPEED_LOCKOUT 3
    else if (RxHeader.StdId == 0x1D3 && RxHeader.DLC == 8)
    {
      if (toyota_checksum(0x1D3, RxData, 8) != RxData[7])
      {
        can_csum_err_cnt ++;
        return;
      }

      low_speed_lockout = (RxData[1] >> 5) & 0x3;
    }
    // ACC control msg on can 0, EON is sending
    else if (RxHeader.StdId == 0x343 && RxHeader.DLC == 8)
    {
      if (toyota_checksum(0x343, RxData, 8) != RxData[7])
      {
        can_csum_err_cnt ++;
        return;
      }

      acc_control_timeout = 0;
    }
  }
  else
  {
    // AEB BRAKING
    if (RxHeader.StdId == 0x344 && RxHeader.DLC == 8)
    {
      if (toyota_checksum(0x344, RxData, 8) != RxData[7])
      {
        can_csum_err_cnt ++;
        return;
      }

      uint16_t stock_aeb = ((RxData[0] << 8U) | RxData[1]) >> 6U;
      stock_aeb_active = (stock_aeb != 0);
    }
    // ACC CONTROL
    else if (RxHeader.StdId == 0x343 && RxHeader.DLC == 8)
    {
      if (!stock_aeb_active)
      {
        if (toyota_checksum(0x343, RxData, 8) != RxData[7])
        {
          can_csum_err_cnt ++;
          return;
        }

        // EON is sending, ignore this msg
        if (acc_control_timeout < MAX_ACC_CONTROL_TIMEOUT)
          return;

#if ENABLE_ACC_INIT_MAGIC
        // initializing, inject fake msg
        if (low_speed_lockout == 3)
        {
          //const uint8_t acc_control_msg[] = { 0x00, 0x00, 0x41, 0x00, 0x00, 0x00, 0x00, 0x8F }; // CH-R init0
          //const uint8_t acc_control_msg[] = { 0x00, 0x00, 0x43, 0x00, 0x00, 0x00, 0x00, 0x91 }; // CH-R init1
          const uint8_t acc_control_msg[] = { 0x00, 0x00, 0x63, 0xC0, 0x00, 0x00, 0x00, 0x71 }; // CH-R SmartDSU
          memcpy(RxData, acc_control_msg, 8);
        }
        else
#endif
        {
          // use stock msg, but update acc type
          RxData[2] &= 0x3F;
          RxData[2] |= 0x40;

          if (car_speed < 45.0)
          {
            // disable lead car to disengage, or disable engagement
            if ((cruise_active && car_speed < 25.0) || ((!cruise_active) && car_speed < 30.0))
            {
              // cmd to 0
              RxData[0] = 0;
              RxData[1] = 0;
              // no lead car
              RxData[2] &= 0xDF;
              // lead standstill to 0
              RxData[3] &= 0xDF;
            }
            // fake moving lead
            else
            {
              // lead car
              RxData[2] |= 0x20;
              // lead standstill to 0
              RxData[3] &= 0xDF;
            }
          }

          // update checksum
          RxData[7] = toyota_checksum(0x343, RxData, 8);
        }
      }
    }
    // SPEED
    else if (RxHeader.StdId == 0xB4 && RxHeader.DLC == 8)
    {
      uint16_t speed = (RxData[5] << 8) | RxData[6];
      car_speed = (float)speed * 0.01;
    }
    // PCM CRUISE
    else if (RxHeader.StdId == 0x1D2 && RxHeader.DLC == 8)
    {
      cruise_active = ((RxData[0] & 0x20) != 0) ? true : false;
    }
  }

  CANMessage to_fwd;
  to_fwd.Size = RxHeader.DLC;
  to_fwd.Id = RxHeader.StdId;
  memcpy(to_fwd.Data, RxData, to_fwd.Size);

  uint8_t fwd_can = (can_number == 0) ? 1 : 0;
  can_send_errs += can_push(can_queues[fwd_can], &to_fwd) ? 0U: 1U;
  process_can(fwd_can);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
  can_rx(CAN_NUM_FROM_CANHANDLE(CanHandle));
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
  can_rx(CAN_NUM_FROM_CANHANDLE(CanHandle));
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
  can_csum_err_cnt = 0;
  can_overflow_cnt = 0;

  low_speed_lockout = 3;
  stock_aeb_active = false;
  car_speed = 0;
  cruise_active = false;

  acc_control_timeout = MAX_ACC_CONTROL_TIMEOUT;

  status_tick_count = 0;
  error_tick_count = 0;

  debug_ring_ready = false;

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
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  /* Configure the CAN Filter */
  CAN_FilterTypeDef sFilterConfig1;
  sFilterConfig1.FilterBank = 14;
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

  puts("can filter starts!\n");
  tx_debug_ring();

  /* Configure the CAN Filter */
  CAN_FilterTypeDef sFilterConfig2;
  sFilterConfig2.FilterBank = 0;
  sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig2.FilterIdHigh = 0x0000;
  sFilterConfig2.FilterIdLow = 0x0000;
  sFilterConfig2.FilterMaskIdHigh = 0x0000;
  sFilterConfig2.FilterMaskIdLow = 0x0000;
  sFilterConfig2.FilterFIFOAssignment = CAN_RX_FIFO0;
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
  while (1)
  {
    /* USER CODE END WHILE */
    // feed the dog
    HAL_IWDG_Refresh(&hiwdg);

    tx_debug_ring();
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
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
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
  hcan2.Init.Prescaler = 4;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
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
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 4095;
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
  htim6.Init.Period = 199;
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
  while (1)
  {
  }
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
