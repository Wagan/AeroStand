/**
  * @file    fmt.h
  * @brief   Форматирование float без printf("%f") (newlib-nano без поддержки float).
  */
#ifndef FMT_H
#define FMT_H

#include <stddef.h>

/**
  * Записать v в buf с dec знаками после точки (0..3). plus != 0: у
  * неотрицательных чисел ставится '+'. "-0.0" не выводится.
  */
void fmt_fixed(char *buf, size_t size, float v, unsigned dec, int plus);

#endif /* FMT_H */
