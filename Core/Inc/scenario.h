/**
  * @file    scenario.h
  * @brief   Сценарий полёта: таймлайн углов трёх осей (ключевые точки).
  *
  * Между ключевыми точками углы интерполируются линейно; точки с одинаковым
  * временем дают мгновенный переход. Сценарий хранится в RAM (до 64 точек),
  * воспроизводится один раз без повтора.
  */
#ifndef SCENARIO_H
#define SCENARIO_H

#include <stdint.h>
#include "stepper.h"

#define SCENARIO_MAX_KEYFRAMES  64U

typedef struct {
  uint32_t time_ms;                    /* время от начала сценария, мс */
  float    angle[STEPPER_AXES];        /* целевые углы осей, градусы */
} scenario_keyframe_t;

typedef enum {
  SCENARIO_OK = 0,
  SCENARIO_ERR_FULL,      /* уже SCENARIO_MAX_KEYFRAMES точек */
  SCENARIO_ERR_EMPTY,     /* сценарий пуст */
  SCENARIO_ERR_VALUE,     /* время не по возрастанию, угол не число */
  SCENARIO_ERR_BUSY,      /* сценарий уже идёт или ось занята движением */
  SCENARIO_ERR_LIMIT      /* угол точки вне пределов оси (см. scenario_get_error_*) */
} scenario_status_t;

/** События для протокола (scenario_take_event). */
#define SCENARIO_EVENT_NONE   0U
#define SCENARIO_EVENT_DONE   1U    /* прошёл все точки и дошёл до конца */
#define SCENARIO_EVENT_ABORT  2U    /* прерван ошибкой (например, драйвер выключен) */

/** Очистить сценарий (нельзя во время воспроизведения). */
scenario_status_t scenario_clear(void);

/** Добавить ключевую точку. time_ms не меньше времени предыдущей точки. */
scenario_status_t scenario_add_keyframe(uint32_t time_ms, float angle0, float angle1, float angle2);

/**
  * Запустить воспроизведение: проверяет пределы всех точек, включает драйверы
  * всех осей (ENABLE) и запоминает момент старта.
  */
scenario_status_t scenario_start(void);

/** Остановить воспроизведение и импульсы осей (драйверы остаются включёнными). */
void scenario_stop(void);

/** 1, пока сценарий выполняется. */
uint8_t scenario_is_running(void);

/** Число загруженных точек. */
uint32_t scenario_count(void);

/** Текущее время сценария, мс (0, если не идёт; не больше времени последней точки). */
uint32_t scenario_get_time_ms(void);

/**
  * Вызывать из while(1). Считает текущие углы интерполяцией и запускает
  * тихие (без "DONE <axis>") движения осей. Занятая ось пропускается: цель
  * пересчитывается на следующем проходе.
  */
void scenario_update(void);

/** Вернуть событие (SCENARIO_EVENT_*) и сбросить его. */
uint8_t scenario_take_event(void);

/** После SCENARIO_ERR_LIMIT: ось и угол, нарушившие предел. */
uint8_t scenario_get_error_axis(void);
float   scenario_get_error_angle(void);

#endif /* SCENARIO_H */
