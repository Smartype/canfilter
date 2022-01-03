// **** libc ****

void delay(uint32_t a) {
  volatile uint32_t i;
  for (i = 0; i < a; i++);
}

void *memset(void *str, int c, unsigned int n) {
  uint8_t *s = str;
  for (unsigned int i = 0; i < n; i++) {
    *s = c;
    s++;
  }
  return str;
}

#define UNALIGNED(X, Y) \
  (((uint32_t)(X) & (sizeof(uint32_t) - 1U)) | ((uint32_t)(Y) & (sizeof(uint32_t) - 1U)))

void *memcpy(void *dest, const void *src, unsigned int len) {
  unsigned int n = len;
  uint8_t *d8 = dest;
  const uint8_t *s8 = src;

  if ((n >= 4U) && !UNALIGNED(s8, d8)) {
    uint32_t *d32 = (uint32_t *)d8; // cppcheck-suppress misra-c2012-11.3 ; already checked that it's properly aligned
    const uint32_t *s32 = (const uint32_t *)s8; // cppcheck-suppress misra-c2012-11.3 ; already checked that it's properly aligned

    while(n >= 16U) {
      *d32 = *s32; d32++; s32++;
      *d32 = *s32; d32++; s32++;
      *d32 = *s32; d32++; s32++;
      *d32 = *s32; d32++; s32++;
      n -= 16U;
    }

    while(n >= 4U) {
      *d32 = *s32; d32++; s32++;
      n -= 4U;
    }

    d8 = (uint8_t *)d32;
    s8 = (const uint8_t *)s32;
  }
  while (n-- > 0U) {
    *d8 = *s8; d8++; s8++;
  }
  return dest;
}

int memcmp(const void* ptr1, const void* ptr2, unsigned int num) {
  const uint8_t *p1 = ptr1;
  const uint8_t *p2 = ptr2;
  const uint32_t end = ((uint32_t)ptr1) + num;
  if ((num >= 4U) && !UNALIGNED(p1, p2)) {
    while (((uint32_t)p1) + 4 <= end) {
      if (*((uint32_t*)p1) != *((uint32_t*)p2)) {
        return -1;
      }
      p1 += 4;
      p2 += 4;
    }
  }

  while (((uint32_t)p1) < end) {
    if (*p1 != *p2) {
      return -1;
    }
    p1 ++;
    p2 ++;
  }

  return 0;
}

void _init() {
}

extern void (*__preinit_array_start []) (void) __attribute__((weak));
extern void (*__preinit_array_end []) (void) __attribute__((weak));
extern void (*__init_array_start []) (void) __attribute__((weak));
extern void (*__init_array_end []) (void) __attribute__((weak));

void __libc_init_array() {
  size_t count, i;

  count = __preinit_array_end - __preinit_array_start;
  for (i = 0; i < count; i++)
    __preinit_array_start[i]();

  _init();

  count = __init_array_end - __init_array_start;
  for (i = 0; i < count; i++)
    __init_array_start[i]();
}
