/**
  * @file    scenario.c
  * @brief   Сценарий полёта: хранение ключевых точек и воспроизведение.
  */
#include "scenario.h"
#include "main.h"

#include <math.h>

/* Частота STEP при движении к текущей цели: чтобы догнать цель примерно за
 * 20 мс, но не меньше 500 Гц (как у ANGLE по умолчанию) и не больше максимума. */
#define SCN_FREQ_MIN_HZ     500UL
#define SCN_FREQ_MAX_HZ     STEPPER_FREQ_MAX
#define SCN_CATCHUP_PER_S   50UL    /* шагов/с на каждый шаг отставания */

static scenario_keyframe_t kf[SCENARIO_MAX_KEYFRAMES];
static uint32_t  kf_count;
static uint8_t   running;
static uint32_t  start_tick;
static uint32_t  cur_time_ms;
static uint8_t   event;
static uint8_t   err_axis;
static float     err_angle;

scenario_status_t scenario_clear(void)
{
  if (running) {
    return SCENARIO_ERR_BUSY;
  }
  kf_count = 0;
  return SCENARIO_OK;
}

scenario_status_t scenario_add_keyframe(uint32_t time_ms, float angle0, float angle1, float angle2)
{
  if (running) {
    return SCENARIO_ERR_BUSY;
  }
  if (!isfinite(angle0) || !isfinite(angle1) || !isfinite(angle2)) {
    return SCENARIO_ERR_VALUE;
  }
  if (kf_count > 0U && time_ms < kf[kf_count - 1U].time_ms) {
    return SCENARIO_ERR_VALUE;
  }
  if (kf_count >= SCENARIO_MAX_KEYFRAMES) {
    return SCENARIO_ERR_FULL;
  }
  kf[kf_count].time_ms  = time_ms;
  kf[kf_count].angle[0] = angle0;
  kf[kf_count].angle[1] = angle1;
  kf[kf_count].angle[2] = angle2;
  kf_count++;
  return SCENARIO_OK;
}

scenario_status_t scenario_start(void)
{
  if (running) {
    return SCENARIO_ERR_BUSY;
  }
  if (kf_count == 0U) {
    return SCENARIO_ERR_EMPTY;
  }
  for (uint8_t a = 0; a < STEPPER_AXES; a++) {
    if (stepper_is_busy(a)) {
      return SCENARIO_ERR_BUSY;
    }
  }
  /* Все точки должны лежать в пределах осей. */
  for (uint32_t i = 0; i < kf_count; i++) {
    for (uint8_t a = 0; a < STEPPER_AXES; a++) {
      float lo, hi;
      stepper_get_limits(a, &lo, &hi);
      if (kf[i].angle[a] < lo || kf[i].angle[a] > hi) {
        err_axis  = a;
        err_angle = kf[i].angle[a];
        return SCENARIO_ERR_LIMIT;
      }
    }
  }
  for (uint8_t a = 0; a < STEPPER_AXES; a++) {
    (void)stepper_enable(a);
  }
  event       = SCENARIO_EVENT_NONE;
  cur_time_ms = 0;
  start_tick  = HAL_GetTick();
  running     = 1;
  return SCENARIO_OK;
}

void scenario_stop(void)
{
  running     = 0;
  cur_time_ms = 0;
  for (uint8_t a = 0; a < STEPPER_AXES; a++) {
    (void)stepper_stop(a);
  }
}

uint8_t scenario_is_running(void)
{
  return running;
}

uint32_t scenario_count(void)
{
  return kf_count;
}

uint32_t scenario_get_time_ms(void)
{
  return running ? cur_time_ms : 0U;
}

uint8_t scenario_take_event(void)
{
  uint8_t e = event;
  event = SCENARIO_EVENT_NONE;
  return e;
}

uint8_t scenario_get_error_axis(void)
{
  return err_axis;
}

float scenario_get_error_angle(void)
{
  return err_angle;
}

/* Углы сценария в момент t (мс): линейная интерполяция между точками. */
static void angles_at(uint32_t t, float out[STEPPER_AXES])
{
  const scenario_keyframe_t *last = &kf[kf_count - 1U];

  if (t <= kf[0].time_ms) {
    for (uint8_t a = 0; a < STEPPER_AXES; a++) { out[a] = kf[0].angle[a]; }
    return;
  }
  if (t >= last->time_ms) {
    for (uint8_t a = 0; a < STEPPER_AXES; a++) { out[a] = last->angle[a]; }
    return;
  }
  /* Последняя точка i с time_ms <= t (при равных временах побеждает поздняя). */
  uint32_t i = 0;
  while (i + 1U < kf_count && kf[i + 1U].time_ms <= t) {
    i++;
  }
  /* t < last.time, значит i+1 существует и kf[i+1].time_ms > t >= kf[i].time_ms. */
  float frac = (float)(t - kf[i].time_ms) / (float)(kf[i + 1U].time_ms - kf[i].time_ms);
  for (uint8_t a = 0; a < STEPPER_AXES; a++) {
    out[a] = kf[i].angle[a] + (kf[i + 1U].angle[a] - kf[i].angle[a]) * frac;
  }
}

void scenario_update(void)
{
  if (!running) {
    return;
  }

  uint32_t t   = HAL_GetTick() - start_tick;
  uint32_t end = kf[kf_count - 1U].time_ms;
  uint8_t  finished_time = (t >= end);
  cur_time_ms = finished_time ? end : t;

  float tgt[STEPPER_AXES];
  angles_at(t, tgt);

  uint8_t all_settled = 1;
  for (uint8_t a = 0; a < STEPPER_AXES; a++) {
    int32_t delta = stepper_steps_to_angle(a, tgt[a]);
    if (stepper_is_busy(a)) {
      all_settled = 0;                 /* предыдущее движение ещё идёт */
      continue;
    }
    if (delta == 0) {
      continue;
    }
    all_settled = 0;
    uint32_t mag  = (uint32_t)((delta > 0) ? delta : -delta);
    uint32_t freq = mag * SCN_CATCHUP_PER_S;
    if (freq < SCN_FREQ_MIN_HZ) { freq = SCN_FREQ_MIN_HZ; }
    if (freq > SCN_FREQ_MAX_HZ) { freq = SCN_FREQ_MAX_HZ; }

    stepper_status_t s = stepper_move_to_angle_quiet(a, tgt[a], freq);
    if (s != STEPPER_OK && s != STEPPER_ERR_BUSY) {
      /* Драйвер выключили, пределы изменили и т.п. - прерываем сценарий. */
      scenario_stop();
      event = SCENARIO_EVENT_ABORT;
      return;
    }
  }

  if (finished_time && all_settled) {
    running     = 0;
    cur_time_ms = 0;
    event       = SCENARIO_EVENT_DONE;
  }
}
