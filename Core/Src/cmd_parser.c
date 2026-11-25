/**
  * @file    cmd_parser.c
  * @brief   Текстовый командный протокол по USB CDC.
  */
#include "cmd_parser.h"
#include "stepper.h"
#include "usbd_cdc_if.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>

extern USBD_HandleTypeDef hUsbDeviceHS;

#define RX_SIZE     256U        /* степень двойки */
#define TX_SIZE     512U        /* степень двойки */
#define TX_CHUNK    64U         /* размер пакета USB FS */
#define MAX_TOKENS  6U

/* ---- Кольцевые буферы (один писатель, один читатель) ---------------------- */

static uint8_t           rx_buf[RX_SIZE];
static volatile uint32_t rx_head;   /* пишет USB-ISR */
static volatile uint32_t rx_tail;   /* читает главный цикл */
static volatile uint8_t  rx_overflow;

static uint8_t           tx_buf[TX_SIZE];
static uint32_t          tx_head;   /* пишет главный цикл */
static uint32_t          tx_tail;
static uint8_t           tx_stage[TX_CHUNK];
static uint16_t          tx_stage_len;   /* >0: пакет в staging ждёт отправки */

/* Текущая строка */
static char     line_buf[CMD_LINE_MAX + 1U];
static uint32_t line_len;
static uint8_t  line_overflow;

void cmd_init(void)
{
  rx_head = rx_tail = 0;
  rx_overflow = 0;
  tx_head = tx_tail = 0;
  tx_stage_len = 0;
  line_len = 0;
  line_overflow = 0;
}

void cmd_rx_push(const uint8_t *data, uint32_t len)
{
  for (uint32_t i = 0; i < len; i++) {
    uint32_t next = (rx_head + 1U) & (RX_SIZE - 1U);
    if (next == rx_tail) {
      rx_overflow = 1;      /* буфер полон: байт теряется */
      return;
    }
    rx_buf[rx_head] = data[i];
    rx_head = next;
  }
}

/* ---- Передача ------------------------------------------------------------- */

static void tx_put(const char *s)
{
  while (*s != '\0') {
    uint32_t next = (tx_head + 1U) & (TX_SIZE - 1U);
    if (next == tx_tail) {
      return;               /* буфер полон: остаток ответа теряется */
    }
    tx_buf[tx_head] = (uint8_t)*s++;
    tx_head = next;
  }
}

static void tx_pump(void)
{
  if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED) {
    /* Хоста нет - накопленные ответы никому не нужны. */
    tx_tail = tx_head;
    tx_stage_len = 0;
    return;
  }
  if (tx_stage_len == 0U) {
    while (tx_tail != tx_head && tx_stage_len < TX_CHUNK) {
      tx_stage[tx_stage_len++] = tx_buf[tx_tail];
      tx_tail = (tx_tail + 1U) & (TX_SIZE - 1U);
    }
  }
  if (tx_stage_len > 0U) {
    if (CDC_Transmit_HS(tx_stage, tx_stage_len) == USBD_OK) {
      tx_stage_len = 0;
    }                       /* USBD_BUSY - повторим в следующем вызове */
  }
}

static void reply_err(const char *text)
{
  tx_put("ERR ");
  tx_put(text);
  tx_put("\n");
}

static void reply_stepper_err(stepper_status_t s)
{
  switch (s) {
    case STEPPER_ERR_AXIS:     reply_err("AXIS");     break;
    case STEPPER_ERR_STEPS:    reply_err("STEPS");    break;
    case STEPPER_ERR_DIR:      reply_err("DIR");      break;
    case STEPPER_ERR_FREQ:     reply_err("FREQ");     break;
    case STEPPER_ERR_BUSY:     reply_err("BUSY");     break;
    case STEPPER_ERR_DISABLED: reply_err("DISABLED"); break;
    default:                   reply_err("INTERNAL"); break;
  }
}

/* ---- Разбор --------------------------------------------------------------- */

/* Целое число строго по всей строке токена. 0 - ошибка. */
static int parse_long(const char *s, long *out)
{
  char *end;
  if (*s == '\0') {
    return 0;
  }
  errno = 0;
  long v = strtol(s, &end, 10);
  if (errno != 0 || *end != '\0') {
    return 0;
  }
  *out = v;
  return 1;
}

static uint32_t split(char *line, char *tok[], uint32_t max)
{
  uint32_t n = 0;
  char *p = line;
  while (*p != '\0') {
    while (*p == ' ' || *p == '\t') {
      *p++ = '\0';
    }
    if (*p == '\0') {
      break;
    }
    if (n == max) {
      return max + 1U;      /* слишком много токенов */
    }
    tok[n++] = p;
    while (*p != '\0' && *p != ' ' && *p != '\t') {
      p++;
    }
  }
  return n;
}

/* Разбор номера оси; при ошибке отвечает ERR и возвращает 0. */
static int get_axis(const char *s, uint8_t *axis)
{
  long v;
  if (!parse_long(s, &v) || v < 0 || v >= (long)STEPPER_AXES) {
    reply_err("AXIS");
    return 0;
  }
  *axis = (uint8_t)v;
  return 1;
}

void cmd_execute(char *line)
{
  char *tok[MAX_TOKENS];
  size_t len = strlen(line);

  while (len > 0U && (line[len - 1U] == '\r' || line[len - 1U] == ' ' || line[len - 1U] == '\t')) {
    line[--len] = '\0';
  }

  uint32_t n = split(line, tok, MAX_TOKENS);
  if (n == 0U) {
    return;                 /* пустая строка - игнорируем */
  }
  if (n > MAX_TOKENS) {
    reply_err("ARGS");
    return;
  }

  for (char *c = tok[0]; *c != '\0'; c++) {
    *c = (char)toupper((unsigned char)*c);
  }
  const char *cmd = tok[0];
  uint8_t axis;

  if (strcmp(cmd, "PING") == 0) {
    if (n != 1U) { reply_err("ARGS"); return; }
    tx_put("PONG\n");
  }
  else if (strcmp(cmd, "STATUS") == 0) {
    if (n != 1U) { reply_err("ARGS"); return; }
    for (uint8_t a = 0; a < STEPPER_AXES; a++) {
      char buf[48];
      (void)snprintf(buf, sizeof(buf), "STATUS %u %u %u %ld\n",
                     (unsigned)a, (unsigned)stepper_is_enabled(a),
                     (unsigned)stepper_is_busy(a), (long)stepper_get_pos(a));
      tx_put(buf);
    }
  }
  else if (strcmp(cmd, "ENABLE") == 0 || strcmp(cmd, "DISABLE") == 0 || strcmp(cmd, "STOP") == 0) {
    if (n != 2U) { reply_err("ARGS"); return; }
    if (!get_axis(tok[1], &axis)) { return; }
    stepper_status_t s;
    if (cmd[0] == 'E') {
      s = stepper_enable(axis);
    } else if (cmd[0] == 'D') {
      s = stepper_disable(axis);
    } else {
      s = stepper_stop(axis);
    }
    if (s == STEPPER_OK) { tx_put("OK\n"); } else { reply_stepper_err(s); }
  }
  else if (strcmp(cmd, "MOVE") == 0) {
    long steps, dir, freq;
    if (n != 5U) { reply_err("ARGS"); return; }
    if (!get_axis(tok[1], &axis)) { return; }
    if (!parse_long(tok[2], &steps) || steps <= 0) { reply_err("STEPS"); return; }
    if (!parse_long(tok[3], &dir) || (dir != 0 && dir != 1)) { reply_err("DIR"); return; }
    if (!parse_long(tok[4], &freq) || freq < (long)STEPPER_FREQ_MIN || freq > (long)STEPPER_FREQ_MAX) {
      reply_err("FREQ");
      return;
    }
    stepper_status_t s = stepper_move(axis, (uint32_t)steps, (uint8_t)dir, (uint32_t)freq);
    if (s == STEPPER_OK) { tx_put("OK\n"); } else { reply_stepper_err(s); }
  }
  else {
    reply_err("UNKNOWN COMMAND");
  }
}

void cmd_poll(void)
{
  /* 1. Принятые байты -> строки. */
  while (rx_tail != rx_head) {
    char c = (char)rx_buf[rx_tail];
    rx_tail = (rx_tail + 1U) & (RX_SIZE - 1U);

    if (c == '\n' || c == '\r') {   /* конец строки: LF, CR или CRLF */
      if (line_overflow) {
        reply_err("LINE TOO LONG");
      } else if (line_len > 0U) {   /* пустая строка (в т.ч. LF после CR) - игнор */
        line_buf[line_len] = '\0';
        cmd_execute(line_buf);
      }
      line_len = 0;
      line_overflow = 0;
    } else if (line_len < CMD_LINE_MAX) {
      line_buf[line_len++] = c;
    } else {
      line_overflow = 1;
    }
  }
  if (rx_overflow) {
    rx_overflow = 0;
    reply_err("RX OVERFLOW");
  }

  /* 2. Завершённые движения. */
  for (uint8_t a = 0; a < STEPPER_AXES; a++) {
    if (stepper_take_done(a)) {
      char buf[16];
      (void)snprintf(buf, sizeof(buf), "DONE %u\n", (unsigned)a);
      tx_put(buf);
    }
  }

  /* 3. Передача. */
  tx_pump();
}
