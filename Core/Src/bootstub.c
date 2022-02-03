#define BOOTSTUB

#define VERS_TAG 0x53524556
#define MIN_VERSION 2
#define CRASH_THRS_BOOTLOADER   20

// ********************* Includes *********************
#include "main.h"
#include "config.h"
#include "critical.h"
#include "libc.h"

#include "early_init.h"

#include "crypto/rsa.h"
#include "crypto/sha.h"

#include "crypto/cert.h"
#include "flasher.h"

#include "stm32f1xx_it.c.h"

void __initialize_hardware_early(void) {
  early_initialization();
}

void fail(void) {
  soft_flasher_start();
}

// know where to sig check
extern void *_app_start[];
extern uint32_t reserved_sram[4];

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

int main(void) {
  // Init interrupt table
  HAL_Init();

  disable_interrupts();

  SystemClock_Config();

  if (enter_bootloader_mode == ENTER_SOFTLOADER_MAGIC) {
    enter_bootloader_mode = 0;
    soft_flasher_start();
  }

  // validate length
  int len = (int)_app_start[0];
  if ((len < 8) || (len > (0x1000000 - 0x4000 - 4 - RSANUMBYTES))) goto fail;

  // compute SHA hash
  uint8_t digest[SHA_DIGEST_SIZE];
  SHA_hash(&_app_start[1], len - 4, digest);

  // verify version, last bytes in the signed area
  uint32_t vers[2] = {0};
  memcpy(&vers, ((void*)&_app_start[0]) + len - sizeof(vers), sizeof(vers));
  if (vers[0] != VERS_TAG || vers[1] < MIN_VERSION) {
    goto fail;
  }

  // allow debug if built from source
  if (RSA_verify(&debug_rsa_key, ((void*)&_app_start[0]) + len, RSANUMBYTES, digest, SHA_DIGEST_SIZE)) {
    goto good;
  }

// here is a failure
fail:
  fail();
  return 0;
good:
  // track app crashes
  if ((reserved_sram[1] & 0xFFFFFF00) != 0xDEADBE00) {
    // uninitialized, set to 1
    reserved_sram[1] = 0xDEADBE01;
  } else {
    // initialized, increase by 1
    uint8_t crashes = (reserved_sram[1] & 0xFF) + 1;
    reserved_sram[1] = 0xDEADBE00 | crashes;
    // app crashed too many times, do not load
    if (crashes >= CRASH_THRS_BOOTLOADER) {
      reserved_sram[1] = 0xDEADBE00;
      fail();
      return 0;
    }
  }

  // jump to flash
  ((void(*)(void)) _app_start[1])();
  return 0;
}

