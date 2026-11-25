/**
  * @file    lcd_status.h
  * @brief   Экран состояния осей на LCD STM32F429I-DISC1 (240x320, портрет).
  *
  * Использует BSP (LTDC + SDRAM + ILI9341). Только отображение, без тача.
  */
#ifndef LCD_STATUS_H
#define LCD_STATUS_H

#include <stdint.h>

#define LCD_STATUS_PERIOD_MS    100U    /* обновление не чаще 10 раз/с */

/**
  * Инициализация LCD (SDRAM, LTDC, ILI9341) и отрисовка статического фрейма.
  * Возвращает 0 при успехе; при ошибке LCD update() становится пустой
  * операцией, остальная прошивка работает.
  */
uint8_t lcd_status_init(void);

/**
  * Обновить динамические поля из данных stepper и состояния USB.
  * Вызывать из while(1); сама ограничивает частоту (HAL_GetTick) и
  * перерисовывает только изменившиеся поля.
  */
void lcd_status_update(void);

#endif /* LCD_STATUS_H */
