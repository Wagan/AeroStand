/**
  * @file    lcd_status.c
  * @brief   Экран состояния осей на LCD (портрет 240x320, Font16/Font20).
  *
  * Раскладка (Font16: 11x16 px, 21 символ в строке):
  *
  *   y=6    "AeroStand v0.1"   (Font20, по центру)
  *   y=30   ----------------
  *   y=36+86*k  Axis k  <ENABLED|DISABLED>  <IDLE|MOVING|--->
  *   y+22       pos:<+000000>            <CW|CCW|-->
  *   y+44       <freq>Hz rem:<N>         (только при движении)
  *   y=284  ----------------
  *   y=292  USB: <Connected|Not configured>
  */
#include "lcd_status.h"
#include "stepper.h"
#include "main.h"
#include "usbd_def.h"
#include "stm32f429i_discovery_lcd.h"

#include <stdio.h>
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceHS;

#define COLOR_BG        LCD_COLOR_BLACK
#define COLOR_TEXT      LCD_COLOR_WHITE
#define COLOR_TITLE     LCD_COLOR_CYAN
#define COLOR_LINE      LCD_COLOR_GRAY
#define COLOR_ON        LCD_COLOR_GREEN
#define COLOR_OFF       LCD_COLOR_GRAY
#define COLOR_MOVING    LCD_COLOR_ORANGE
#define COLOR_WARN      LCD_COLOR_ORANGE

#define FONT            (&Font16)
#define CHAR_W          11U
#define LINE1_DY        0U
#define LINE2_DY        22U
#define LINE3_DY        44U
#define AXIS_Y0         36U
#define AXIS_PITCH      86U
#define SEP_TOP_Y       30U
#define SEP_BOTTOM_Y    284U
#define USB_Y           292U
#define MARGIN_X        4U

#define X_DRV           (MARGIN_X + 7U * CHAR_W)     /* после "Axis k " */
#define X_MOT           (MARGIN_X + 16U * CHAR_W)    /* после "Axis k ENABLED " */
#define X_POS           (MARGIN_X + 4U * CHAR_W)     /* после "pos:" */
#define X_DIR           (MARGIN_X + 17U * CHAR_W)
#define X_USB           (MARGIN_X + 5U * CHAR_W)     /* после "USB: " */

#define FIELD_MAX       24U

/* Кэш показанного значения поля: перерисовка только при изменении. */
typedef struct {
  char     text[FIELD_MAX + 1U];
  uint32_t color;
  uint8_t  valid;
} field_t;

/* Поля оси: драйвер, движение, позиция, направление, freq/rem. */
enum { F_DRV, F_MOT, F_POS, F_DIR, F_RUN, F_COUNT };

static field_t  axis_f[STEPPER_AXES][F_COUNT];
static field_t  usb_f;
static uint8_t  lcd_ok;
static uint32_t last_tick;

/**
  * Вывести текст в фиксированное поле шириной width символов (дополняется
  * пробелами - затирает прежнее содержимое), только если изменился текст/цвет.
  */
static void draw_field(field_t *f, uint16_t x, uint16_t y, uint8_t width,
                       uint32_t color, const char *text)
{
  char buf[FIELD_MAX + 1U];
  size_t n = strlen(text);
  if (width > FIELD_MAX) {
    width = FIELD_MAX;
  }
  if (n > width) {
    n = width;
  }
  memcpy(buf, text, n);
  memset(&buf[n], ' ', width - n);
  buf[width] = '\0';

  if (f->valid && f->color == color && strcmp(f->text, buf) == 0) {
    return;
  }
  BSP_LCD_SetTextColor(color);
  BSP_LCD_DisplayStringAt(x, y, (uint8_t *)buf, LEFT_MODE);
  strcpy(f->text, buf);
  f->color = color;
  f->valid = 1;
}

static void draw_static_text(uint16_t x, uint16_t y, uint32_t color, const char *text)
{
  BSP_LCD_SetTextColor(color);
  BSP_LCD_DisplayStringAt(x, y, (uint8_t *)text, LEFT_MODE);
}

static void draw_frame(void)
{
  BSP_LCD_Clear(COLOR_BG);
  BSP_LCD_SetBackColor(COLOR_BG);

  BSP_LCD_SetFont(&Font20);
  BSP_LCD_SetTextColor(COLOR_TITLE);
  BSP_LCD_DisplayStringAt(0, 6, (uint8_t *)"AeroStand v0.1", CENTER_MODE);

  BSP_LCD_SetFont(FONT);
  BSP_LCD_SetTextColor(COLOR_LINE);
  BSP_LCD_DrawHLine(0, SEP_TOP_Y, BSP_LCD_GetXSize());
  BSP_LCD_DrawHLine(0, SEP_BOTTOM_Y, BSP_LCD_GetXSize());

  for (uint8_t a = 0; a < STEPPER_AXES; a++) {
    uint16_t y = (uint16_t)(AXIS_Y0 + a * AXIS_PITCH);
    char label[8];
    (void)snprintf(label, sizeof(label), "Axis %u", (unsigned)a);
    draw_static_text(MARGIN_X, y + LINE1_DY, COLOR_TEXT, label);
    draw_static_text(MARGIN_X, y + LINE2_DY, COLOR_TEXT, "pos:");
  }
  draw_static_text(MARGIN_X, USB_Y, COLOR_TEXT, "USB:");

  memset(axis_f, 0, sizeof(axis_f));
  memset(&usb_f, 0, sizeof(usb_f));
}

uint8_t lcd_status_init(void)
{
  if (BSP_LCD_Init() != LCD_OK) {
    lcd_ok = 0;
    return 1;
  }
  BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER);
  BSP_LCD_SelectLayer(0);
  BSP_LCD_DisplayOn();
  draw_frame();
  lcd_ok = 1;
  last_tick = HAL_GetTick() - LCD_STATUS_PERIOD_MS;
  lcd_status_update();      /* первая отрисовка динамических полей сразу */
  return 0;
}

static void update_axis(uint8_t a)
{
  uint16_t y = (uint16_t)(AXIS_Y0 + a * AXIS_PITCH);
  char buf[FIELD_MAX + 1U];
  uint8_t enabled = stepper_is_enabled(a);
  uint8_t busy    = stepper_is_busy(a);

  /* Драйвер и движение. */
  draw_field(&axis_f[a][F_DRV], X_DRV, y + LINE1_DY, 8U,
             enabled ? COLOR_ON : COLOR_OFF, enabled ? "ENABLED" : "DISABLED");
  if (!enabled) {
    draw_field(&axis_f[a][F_MOT], X_MOT, y + LINE1_DY, 6U, COLOR_OFF, "---");
  } else if (busy) {
    draw_field(&axis_f[a][F_MOT], X_MOT, y + LINE1_DY, 6U, COLOR_MOVING, "MOVING");
  } else {
    draw_field(&axis_f[a][F_MOT], X_MOT, y + LINE1_DY, 6U, COLOR_TEXT, "IDLE");
  }

  /* Позиция и направление (направление - последнего движения; "--" если
   * драйвер выключен или движений с момента включения не было). */
  (void)snprintf(buf, sizeof(buf), "%+06ld", (long)stepper_get_pos(a));
  draw_field(&axis_f[a][F_POS], X_POS, y + LINE2_DY, 11U,
             enabled ? COLOR_TEXT : COLOR_OFF, buf);
  if (enabled && stepper_has_dir(a)) {
    draw_field(&axis_f[a][F_DIR], X_DIR, y + LINE2_DY, 3U, COLOR_TEXT,
               stepper_get_dir(a) ? "CW" : "CCW");
  } else {
    draw_field(&axis_f[a][F_DIR], X_DIR, y + LINE2_DY, 3U, COLOR_OFF, "--");
  }

  /* Частота и остаток шагов - только пока ось движется. */
  if (busy) {
    (void)snprintf(buf, sizeof(buf), "%luHz rem:%lu",
                   (unsigned long)stepper_get_freq(a),
                   (unsigned long)stepper_get_remaining(a));
    draw_field(&axis_f[a][F_RUN], MARGIN_X, y + LINE3_DY, 21U, COLOR_MOVING, buf);
  } else {
    draw_field(&axis_f[a][F_RUN], MARGIN_X, y + LINE3_DY, 21U, COLOR_TEXT, "");
  }
}

void lcd_status_update(void)
{
  if (!lcd_ok) {
    return;
  }
  uint32_t now = HAL_GetTick();
  if ((now - last_tick) < LCD_STATUS_PERIOD_MS) {
    return;
  }
  last_tick = now;

  BSP_LCD_SetFont(FONT);
  BSP_LCD_SetBackColor(COLOR_BG);
  for (uint8_t a = 0; a < STEPPER_AXES; a++) {
    update_axis(a);
  }

  if (hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED) {
    draw_field(&usb_f, X_USB, USB_Y, 14U, COLOR_ON, "Connected");
  } else {
    draw_field(&usb_f, X_USB, USB_Y, 14U, COLOR_WARN, "Not configured");
  }
}
