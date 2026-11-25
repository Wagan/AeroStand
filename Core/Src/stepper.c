/**
  * @file    stepper.c
  * @brief   Драйвер шаговых двигателей (STEP/DIR/EN), 3 оси, полный шаг.
  */
#include "stepper.h"
#include "main.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim9;

/* Тактовая частота таймеров, МГц (тактирование 168 МГц):
 * TIM3/TIM4 на APB1 = 84 МГц, TIM9 на APB2 = 168 МГц. */
#define TIM_CLK_APB1_MHZ    84UL
#define TIM_CLK_APB2_MHZ    168UL

/* Счётчики 16-битные. Два диапазона тика:
 *  freq >= 16 Гц: тик 1 МГц  (ARR = 1e6/f - 1 <= 62499; на 5 кГц ARR = 199)
 *  freq <  16 Гц: тик 10 кГц (ARR = 1e4/f - 1 <= 9999) */
#define TICK_FAST_HZ        1000000UL
#define TICK_SLOW_HZ        10000UL
#define FREQ_SLOW_BELOW     16UL

#define STEPS_MAX           0x7FFFFFFFUL

typedef struct {
  TIM_HandleTypeDef *htim;
  uint32_t           channel;
  uint32_t           active_channel;   /* HAL_TIM_ACTIVE_CHANNEL_x */
  uint32_t           cc_flag;          /* TIM_FLAG_CCx */
  uint32_t           tim_clk_mhz;
  GPIO_TypeDef      *dir_port;
  uint16_t           dir_pin;
  GPIO_TypeDef      *en_port;
  uint16_t           en_pin;
} axis_hw_t;

typedef struct {
  volatile uint32_t remaining;   /* осталось импульсов */
  volatile int32_t  pos;         /* позиция, шаги */
  volatile uint8_t  busy;
  volatile uint8_t  done;        /* флаг штатного завершения */
  volatile uint8_t  dir;
  volatile uint8_t  dir_valid;   /* было движение с момента включения */
  volatile uint32_t freq;        /* Гц последнего движения */
  uint8_t           enabled;
} axis_state_t;

static const axis_hw_t hw[STEPPER_AXES] = {
  { &htim3, TIM_CHANNEL_1, HAL_TIM_ACTIVE_CHANNEL_1, TIM_FLAG_CC1, TIM_CLK_APB1_MHZ,
    DIR0_GPIO_Port, DIR0_Pin, EN0_GPIO_Port, EN0_Pin },
  { &htim4, TIM_CHANNEL_2, HAL_TIM_ACTIVE_CHANNEL_2, TIM_FLAG_CC2, TIM_CLK_APB1_MHZ,
    DIR1_GPIO_Port, DIR1_Pin, EN1_GPIO_Port, EN1_Pin },
  { &htim9, TIM_CHANNEL_1, HAL_TIM_ACTIVE_CHANNEL_1, TIM_FLAG_CC1, TIM_CLK_APB2_MHZ,
    DIR2_GPIO_Port, DIR2_Pin, EN2_GPIO_Port, EN2_Pin },
};

static axis_state_t st[STEPPER_AXES];

static void channel_stop(uint8_t axis)
{
  /* Выключает прерывание CC, выход канала и (если нет других каналов) счётчик.
   * Выключенный канал отдаёт пин обратно GPIO (ODR=0) -> STEP=LOW. */
  (void)HAL_TIM_PWM_Stop_IT(hw[axis].htim, hw[axis].channel);
}

void stepper_init(void)
{
  for (uint8_t a = 0; a < STEPPER_AXES; a++) {
    channel_stop(a);
    HAL_GPIO_WritePin(hw[a].dir_port, hw[a].dir_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hw[a].en_port, hw[a].en_pin, GPIO_PIN_SET);   /* выключен */
    st[a].remaining = 0;
    st[a].pos       = 0;
    st[a].busy      = 0;
    st[a].done      = 0;
    st[a].dir       = 0;
    st[a].dir_valid = 0;
    st[a].freq      = 0;
    st[a].enabled   = 0;
  }
}

stepper_status_t stepper_enable(uint8_t axis)
{
  if (axis >= STEPPER_AXES) {
    return STEPPER_ERR_AXIS;
  }
  HAL_GPIO_WritePin(hw[axis].en_port, hw[axis].en_pin, GPIO_PIN_RESET); /* активный LOW */
  st[axis].enabled = 1;
  return STEPPER_OK;
}

stepper_status_t stepper_disable(uint8_t axis)
{
  if (axis >= STEPPER_AXES) {
    return STEPPER_ERR_AXIS;
  }
  (void)stepper_stop(axis);
  HAL_GPIO_WritePin(hw[axis].en_port, hw[axis].en_pin, GPIO_PIN_SET);
  st[axis].enabled = 0;
  st[axis].dir_valid = 0;
  return STEPPER_OK;
}

stepper_status_t stepper_move(uint8_t axis, uint32_t steps, uint8_t dir, uint32_t freq_hz)
{
  if (axis >= STEPPER_AXES) {
    return STEPPER_ERR_AXIS;
  }
  if (steps == 0U || steps > STEPS_MAX) {
    return STEPPER_ERR_STEPS;
  }
  if (dir > 1U) {
    return STEPPER_ERR_DIR;
  }
  if (freq_hz < STEPPER_FREQ_MIN || freq_hz > STEPPER_FREQ_MAX) {
    return STEPPER_ERR_FREQ;
  }
  if (!st[axis].enabled) {
    return STEPPER_ERR_DISABLED;
  }
  if (st[axis].busy) {
    return STEPPER_ERR_BUSY;
  }

  const axis_hw_t *h = &hw[axis];
  TIM_HandleTypeDef *t = h->htim;

  /* Период в тиках выбранного диапазона. */
  uint32_t tick_hz = (freq_hz < FREQ_SLOW_BELOW) ? TICK_SLOW_HZ : TICK_FAST_HZ;
  uint32_t psc     = (h->tim_clk_mhz * 1000000UL) / tick_hz - 1UL;
  uint32_t period  = (tick_hz + freq_hz / 2UL) / freq_hz;     /* с округлением */
  uint32_t arr     = period - 1UL;
  uint32_t ccr     = period / 2UL;                            /* скважность ~50% */

  /* Направление до первого импульса (MP8825: setup DIR >= 200 нс). */
  HAL_GPIO_WritePin(h->dir_port, h->dir_pin, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
  for (volatile uint32_t i = 0; i < 50U; i++) {
    __NOP();                                                  /* ~1 мкс @168 МГц */
  }

  st[axis].dir       = dir;
  st[axis].dir_valid = 1;
  st[axis].freq      = freq_hz;
  st[axis].remaining = steps;
  st[axis].done      = 0;
  st[axis].busy      = 1;

  /* Канал выключен: безопасно менять PSC/ARR/CCR и сбросить счётчик (UG). */
  __HAL_TIM_SET_PRESCALER(t, psc);
  __HAL_TIM_SET_AUTORELOAD(t, arr);
  __HAL_TIM_SET_COMPARE(t, h->channel, ccr);
  __HAL_TIM_SET_COUNTER(t, 0U);
  (void)HAL_TIM_GenerateEvent(t, TIM_EVENTSOURCE_UPDATE);     /* загрузить PSC */
  __HAL_TIM_CLEAR_FLAG(t, TIM_FLAG_UPDATE | h->cc_flag);

  /* Старт: выход сразу HIGH (CNT=0 < CCR), спад по совпадению с CCR = 1 шаг. */
  if (HAL_TIM_PWM_Start_IT(t, h->channel) != HAL_OK) {
    st[axis].busy      = 0;
    st[axis].remaining = 0;
    return STEPPER_ERR_BUSY;
  }
  return STEPPER_OK;
}

uint8_t stepper_is_busy(uint8_t axis)
{
  return (axis < STEPPER_AXES) ? st[axis].busy : 0U;
}

stepper_status_t stepper_stop(uint8_t axis)
{
  if (axis >= STEPPER_AXES) {
    return STEPPER_ERR_AXIS;
  }
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (st[axis].busy) {
    channel_stop(axis);
    st[axis].remaining = 0;
    st[axis].busy      = 0;
    st[axis].done      = 0;   /* аварийная остановка - не штатное завершение */
  }
  __set_PRIMASK(primask);
  return STEPPER_OK;
}

uint8_t stepper_is_enabled(uint8_t axis)
{
  return (axis < STEPPER_AXES) ? st[axis].enabled : 0U;
}

int32_t stepper_get_pos(uint8_t axis)
{
  return (axis < STEPPER_AXES) ? st[axis].pos : 0;
}

uint32_t stepper_get_freq(uint8_t axis)
{
  return (axis < STEPPER_AXES) ? st[axis].freq : 0U;
}

uint32_t stepper_get_remaining(uint8_t axis)
{
  return (axis < STEPPER_AXES) ? st[axis].remaining : 0U;
}

uint8_t stepper_get_dir(uint8_t axis)
{
  return (axis < STEPPER_AXES) ? st[axis].dir : 0U;
}

uint8_t stepper_has_dir(uint8_t axis)
{
  return (axis < STEPPER_AXES) ? st[axis].dir_valid : 0U;
}

uint8_t stepper_take_done(uint8_t axis)
{
  uint8_t d = 0;
  if (axis < STEPPER_AXES) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    d = st[axis].done;
    st[axis].done = 0;
    __set_PRIMASK(primask);
  }
  return d;
}

/**
  * Прерывание Capture/Compare: спад STEP, т.е. импульс отработан.
  * Переопределяет weak-функцию HAL (вызывается из HAL_TIM_IRQHandler).
  */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  for (uint8_t a = 0; a < STEPPER_AXES; a++) {
    if (hw[a].htim != htim || htim->Channel != hw[a].active_channel || !st[a].busy) {
      continue;
    }
    st[a].pos += st[a].dir ? 1 : -1;
    if (st[a].remaining > 0U) {
      st[a].remaining--;
    }
    if (st[a].remaining == 0U) {
      channel_stop(a);
      st[a].busy = 0;
      st[a].done = 1;
    }
  }
}
