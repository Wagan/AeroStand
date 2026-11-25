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
  STEPPER_ERR_DISABLED,   /* драйвер оси выключен */
  STEPPER_ERR_LIMIT,      /* угол вне допустимого диапазона */
  STEPPER_ERR_PARAM       /* неверный параметр калибровки/пределов */
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

/** Частота последнего запущенного движения, Гц (0, если движений не было). */
uint32_t stepper_get_freq(uint8_t axis);

/** Сколько импульсов осталось до конца текущего движения. */
uint32_t stepper_get_remaining(uint8_t axis);

/**
  * Направление последнего движения (1 = CW/pos растёт, 0 = CCW).
  * Результат имеет смысл, только если stepper_has_dir() == 1.
  */
uint8_t stepper_get_dir(uint8_t axis);

/** 1, если с момента включения драйвера оси было хотя бы одно движение. */
uint8_t stepper_has_dir(uint8_t axis);

/* ---- Углы и калибровка ---------------------------------------------------- */

/**
  * Коэффициент пересчёта, шагов на градус (> 0, конечный). По умолчанию 1.0.
  * Позиция в шагах не меняется, меняется только пересчёт в угол.
  */
stepper_status_t stepper_set_calibration(uint8_t axis, float steps_per_degree);

/** Допустимый диапазон углов, градусы (min < max). По умолчанию -90..+90. */
stepper_status_t stepper_set_limits(uint8_t axis, float min_deg, float max_deg);

/** Текущую позицию считать нулём (0 град). Ось не должна двигаться. */
stepper_status_t stepper_set_zero(uint8_t axis);

/**
  * Переместить ось в угол angle_deg: цель в шагах = angle_deg * steps_per_degree
  * + zero_offset, движение на разность с текущей позицией. Если угол вне
  * [min, max] - STEPPER_ERR_LIMIT, ось не двигается. Если ось уже в цели -
  * STEPPER_OK без импульсов (флаг "завершено" выставляется сразу).
  */
stepper_status_t stepper_move_to_angle(uint8_t axis, float angle_deg, uint32_t freq_hz);

/** Текущий угол, градусы: (pos - zero_offset) / steps_per_degree. */
float stepper_get_angle(uint8_t axis);

/** Калибровочный коэффициент и пределы (для ответов об ошибках). */
float stepper_get_calibration(uint8_t axis);
void  stepper_get_limits(uint8_t axis, float *min_deg, float *max_deg);

#endif /* STEPPER_H */
