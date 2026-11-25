/**
  * @file    stepper.h
  * @brief   Драйвер шаговых двигателей (STEP/DIR/EN), 3 оси, полный шаг.
  *
  * STEP формируется таймером в режиме PWM (скважность ~50%): один период =
  * один импульс STEP = один шаг. Импульсы считаются в прерывании по
  * Capture/Compare (спад импульса), после последнего шага канал отключается.
  */
#ifndef STEPPER_H
#define STEPPER_H

#include <stdint.h>

#define STEPPER_AXES        3U
#define STEPPER_FREQ_MIN    1U      /* Гц */
#define STEPPER_FREQ_MAX    5000U   /* Гц */

typedef enum {
  STEPPER_OK = 0,
  STEPPER_ERR_AXIS,       /* неверный номер оси */
  STEPPER_ERR_STEPS,      /* steps == 0 или слишком много */
  STEPPER_ERR_DIR,        /* dir не 0/1 */
  STEPPER_ERR_FREQ,       /* freq вне 1..5000 */
  STEPPER_ERR_BUSY,       /* ось уже движется */
  STEPPER_ERR_DISABLED    /* драйвер оси выключен */
} stepper_status_t;

/** Инициализация всех осей: EN=HIGH (драйвер выключен), pos=0. */
void stepper_init(void);

/** EN=LOW (драйвер включён). */
stepper_status_t stepper_enable(uint8_t axis);

/** Остановить движение (если идёт) и EN=HIGH (драйвер выключен). */
stepper_status_t stepper_disable(uint8_t axis);

/**
  * Выдать steps импульсов STEP с частотой freq_hz, DIR=dir (0/1).
  * Ось должна быть включена и свободна. Неблокирующая.
  */
stepper_status_t stepper_move(uint8_t axis, uint32_t steps, uint8_t dir, uint32_t freq_hz);

/** 1, пока ось движется (0 при неверной оси). */
uint8_t stepper_is_busy(uint8_t axis);

/** Аварийная остановка: прекратить импульсы. Драйвер остаётся включённым. */
stepper_status_t stepper_stop(uint8_t axis);

/** 1, если драйвер оси включён. */
uint8_t stepper_is_enabled(uint8_t axis);

/** Текущая позиция в шагах: dir=1 прибавляет, dir=0 убавляет. */
int32_t stepper_get_pos(uint8_t axis);

/**
  * Вернуть 1 и сбросить флаг, если движение оси завершилось штатно
  * (все шаги отработаны) с момента прошлого вызова. После STOP не
  * устанавливается.
  */
uint8_t stepper_take_done(uint8_t axis);

#endif /* STEPPER_H */
