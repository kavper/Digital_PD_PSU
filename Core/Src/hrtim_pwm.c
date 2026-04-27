#include "hrtim_pwm.h"

static HRTIM_HandleTypeDef *pwm_hrtim = 0;
static float pwm_duty = HRTIM_PWM_DUTY_MIN;
static uint8_t pwm_adc_sample_high_side_window = 0U;

#define HRTIM_PWM_ADC_LOW_TO_HIGH_TICKS \
  ((HRTIM_PWM_PERIOD_TICKS * 55UL) / 100UL)
#define HRTIM_PWM_ADC_HIGH_TO_LOW_TICKS \
  ((HRTIM_PWM_PERIOD_TICKS * 45UL) / 100UL)

static float pwm_clamp(float value, float min_value, float max_value)
{
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

static uint32_t pwm_adc_trigger_clamp(uint32_t cmp3)
{
  if (cmp3 >= (HRTIM_PWM_PERIOD_TICKS - HRTIM_PWM_MIN_EVENT_GAP_TICKS)) {
    cmp3 = HRTIM_PWM_PERIOD_TICKS - HRTIM_PWM_MIN_EVENT_GAP_TICKS;
  }
  if (cmp3 < HRTIM_PWM_MIN_EVENT_GAP_TICKS) {
    cmp3 = HRTIM_PWM_MIN_EVENT_GAP_TICKS;
  }

  return cmp3;
}

static uint32_t pwm_adc_trigger_from_cmp1(uint32_t cmp1)
{
  if ((cmp1 == 0UL) || (cmp1 >= HRTIM_PWM_PERIOD_TICKS)) {
    return pwm_adc_trigger_clamp(HRTIM_PWM_PERIOD_TICKS / 2UL);
  }

  /*
   * Sample in the middle of the longer quiet PWM window.
   *
   * TD2/PB15 high-side is active from period event to CMP1.
   * TD1/PB14 low-side is active from CMP1 to period event.
   *
   * For duty <~50% the low-side/off window is longer, so trigger after CMP1.
   * For duty >~50% the high-side/on window is longer, so trigger before CMP1.
   * 45/55% hysteresis avoids ADC phase chatter around exactly 50% duty.
   */
  if (pwm_adc_sample_high_side_window != 0U) {
    if (cmp1 < HRTIM_PWM_ADC_HIGH_TO_LOW_TICKS) {
      pwm_adc_sample_high_side_window = 0U;
    }
  } else if (cmp1 > HRTIM_PWM_ADC_LOW_TO_HIGH_TICKS) {
    pwm_adc_sample_high_side_window = 1U;
  }

  if (pwm_adc_sample_high_side_window != 0U) {
    return pwm_adc_trigger_clamp(cmp1 / 2UL);
  }

  return pwm_adc_trigger_clamp(cmp1 + ((HRTIM_PWM_PERIOD_TICKS - cmp1) / 2UL));
}

uint32_t HRTIM_PWM_DutyToTicks(float duty)
{
  const float clamped = pwm_clamp(duty, HRTIM_PWM_DUTY_MIN, HRTIM_PWM_DUTY_MAX);

  if (clamped <= 0.0f) {
    return 0UL;
  }

  uint32_t ticks = (uint32_t)(clamped * (float)HRTIM_PWM_PERIOD_TICKS);

  if (ticks < HRTIM_PWM_MIN_EVENT_GAP_TICKS) {
    ticks = HRTIM_PWM_MIN_EVENT_GAP_TICKS;
  }
  if (ticks > (HRTIM_PWM_PERIOD_TICKS - HRTIM_PWM_MIN_EVENT_GAP_TICKS)) {
    ticks = HRTIM_PWM_PERIOD_TICKS - HRTIM_PWM_MIN_EVENT_GAP_TICKS;
  }

  return ticks;
}

uint32_t HRTIM_PWM_GetPeriodTicks(void)
{
  return HRTIM_PWM_PERIOD_TICKS;
}

void HRTIM_PWM_Init(HRTIM_HandleTypeDef *hhrtim)
{
  pwm_hrtim = hhrtim;
  pwm_duty = HRTIM_PWM_DUTY_MIN;
  pwm_adc_sample_high_side_window = 0U;
  HRTIM_PWM_SetDuty(pwm_duty);

  if (pwm_hrtim != 0) {
    (void)HAL_HRTIM_SoftwareUpdate(pwm_hrtim, HRTIM_TIMERUPDATE_D);
  }
}

void HRTIM_PWM_StartCounter(void)
{
  if (pwm_hrtim == 0) {
    return;
  }

  (void)HAL_HRTIM_WaveformCounterStart(pwm_hrtim, HRTIM_TIMERID_TIMER_D);
}

void HRTIM_PWM_StopCounter(void)
{
  if (pwm_hrtim == 0) {
    return;
  }

  (void)HAL_HRTIM_WaveformCounterStop(pwm_hrtim, HRTIM_TIMERID_TIMER_D);
}

void HRTIM_PWM_EnableOutputs(void)
{
  if (pwm_hrtim == 0) {
    return;
  }

  (void)HAL_HRTIM_WaveformOutputStart(pwm_hrtim, HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);
}

void HRTIM_PWM_DisableOutputs(void)
{
  if (pwm_hrtim == 0) {
    return;
  }

  (void)HAL_HRTIM_WaveformOutputStop(pwm_hrtim, HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);
}

void HRTIM_PWM_EnableLowSideOnly(void)
{
  if (pwm_hrtim == 0) {
    return;
  }

  /* Used only for explicit 0 V hold: high-side off, low-side pulls to GND. */
  (void)HAL_HRTIM_WaveformOutputStop(pwm_hrtim, HRTIM_OUTPUT_TD2);
  (void)HAL_HRTIM_WaveformOutputStart(pwm_hrtim, HRTIM_OUTPUT_TD1);
}

void HRTIM_PWM_ForceUpdate(void)
{
  if (pwm_hrtim == 0) {
    return;
  }

  (void)HAL_HRTIM_SoftwareUpdate(pwm_hrtim, HRTIM_TIMERUPDATE_D);
}

void HRTIM_PWM_Start(void)
{
  if (pwm_hrtim == 0) {
    return;
  }

  HRTIM_PWM_StartCounter();
  HRTIM_PWM_EnableOutputs();
}

void HRTIM_PWM_Stop(void)
{
  if (pwm_hrtim == 0) {
    return;
  }

  HRTIM_PWM_SetDuty(HRTIM_PWM_DUTY_MIN);
  (void)HAL_HRTIM_SoftwareUpdate(pwm_hrtim, HRTIM_TIMERUPDATE_D);
  HRTIM_PWM_DisableOutputs();
  HRTIM_PWM_StopCounter();
}

void HRTIM_PWM_SetDuty(float duty)
{
  if (pwm_hrtim == 0) {
    return;
  }

  pwm_duty = pwm_clamp(duty, HRTIM_PWM_DUTY_MIN, HRTIM_PWM_DUTY_MAX);

  const uint32_t cmp1 = HRTIM_PWM_DutyToTicks(pwm_duty);
  const uint32_t cmp3 = pwm_adc_trigger_from_cmp1(cmp1);

  pwm_hrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = cmp1;
  pwm_hrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP3xR = cmp3;
}

float HRTIM_PWM_GetDuty(void)
{
  return pwm_duty;
}
