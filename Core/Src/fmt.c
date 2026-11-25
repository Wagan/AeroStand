/**
  * @file    fmt.c
  * @brief   Форматирование float без printf("%f").
  */
#include "fmt.h"

#include <stdio.h>

void fmt_fixed(char *buf, size_t size, float v, unsigned dec, int plus)
{
  static const long pow10[4] = { 1L, 10L, 100L, 1000L };
  if (dec > 3U) {
    dec = 3U;
  }
  long scale = pow10[dec];
  int neg = (v < 0.0f);
  float av = neg ? -v : v;
  if (av > 1.0e6f) {
    av = 1.0e6f;                    /* защита от переполнения long */
  }
  long q = (long)(av * (float)scale + 0.5f);
  if (q == 0) {
    neg = 0;                        /* не печатаем "-0.0" */
  }
  const char *sign = neg ? "-" : (plus ? "+" : "");
  if (dec == 0U) {
    (void)snprintf(buf, size, "%s%ld", sign, q);
  } else {
    (void)snprintf(buf, size, "%s%ld.%0*ld", sign, q / scale, (int)dec, q % scale);
  }
}
