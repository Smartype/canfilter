// Early bringup
#define ENTER_BOOTLOADER_MAGIC 0xdeadbeefU
#define ENTER_SOFTLOADER_MAGIC 0xdeadc0deU
#define BOOT_NORMAL 0xdeadb111U

extern void *g_pfnVectors;
extern uint32_t enter_bootloader_mode;

void jump_to_bootloader(void) {
  // do enter bootloader
  enter_bootloader_mode = 0;
  void (*bootloader)(void) = (void (*)(void)) (*((uint32_t *)BOOTLOADER_ADDRESS));

  // jump to bootloader
  enable_interrupts();
  bootloader();

  // reset on exit
  enter_bootloader_mode = BOOT_NORMAL;
  NVIC_SystemReset();
}

static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
}

void early_initialization(void) {
  // Reset global critical depth
  disable_interrupts();
  global_critical_depth = 0;

  // after it's been in the bootloader, things are initted differently, so we reset
  if ((enter_bootloader_mode != BOOT_NORMAL) &&
      (enter_bootloader_mode != ENTER_BOOTLOADER_MAGIC) &&
      (enter_bootloader_mode != ENTER_SOFTLOADER_MAGIC)) {
    enter_bootloader_mode = BOOT_NORMAL;
    NVIC_SystemReset();
  }

  // if wrong chip, reboot
  volatile unsigned int id = DBGMCU->IDCODE;
    if ((id & 0xFFFU) != MCU_IDCODE) {
      enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
    }

  // setup interrupt table
  SCB->VTOR = (uint32_t)&g_pfnVectors;

  HAL_Init();

  // early GPIOs float everything
  MX_GPIO_Init();

  if (enter_bootloader_mode == ENTER_BOOTLOADER_MAGIC) {
    jump_to_bootloader();
  }
}

