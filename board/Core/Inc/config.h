#define MCU_IDCODE 0x418U

// from the linker script
#define APP_START_ADDRESS 0x8004000U

#define CORE_FREQ 72U // in Mhz
//APB1 - 38Mhz, APB2 - 72Mhz
#define APB1_FREQ CORE_FREQ/2U 
#define APB2_FREQ CORE_FREQ/1U

#define BOOTLOADER_ADDRESS 0x1FFF8004U

#ifndef NULL
#define NULL ((void*)0)
#define COMPILE_TIME_ASSERT(pred) ((void)sizeof(char[1 - (2 * ((int)(!(pred))))]))
#endif

#define MIN(a,b) \
 ({ __typeof__ (a) _a = (a); \
     __typeof__ (b) _b = (b); \
   (_a < _b) ? _a : _b; })

#define MAX(a,b) \
 ({ __typeof__ (a) _a = (a); \
     __typeof__ (b) _b = (b); \
   (_a > _b) ? _a : _b; })

#define ABS(a) \
 ({ __typeof__ (a) _a = (a); \
   (_a > 0) ? _a : (-_a); })

#define MAX_RESP_LEN 0x40U

#define GET_BUS(msg) (((msg)->RDTR >> 4) & 0xFF)
#define GET_LEN(msg) ((msg)->RDTR & 0xF)
#define GET_ADDR(msg) ((((msg)->RIR & 4) != 0) ? ((msg)->RIR >> 3) : ((msg)->RIR >> 21))
#define GET_BYTE(msg, b) (((int)(b) > 3) ? (((msg)->RDHR >> (8U * ((unsigned int)(b) % 4U))) & 0xFFU) : (((msg)->RDLR >> (8U * (unsigned int)(b))) & 0xFFU))
#define GET_BYTES_04(msg) ((msg)->RDLR)
#define GET_BYTES_48(msg) ((msg)->RDHR)
#define GET_FLAG(value, mask) (((__typeof__(mask))(value) & (mask)) == (mask))

#define CAN_INIT_TIMEOUT_MS 500U

#include <stdbool.h>

