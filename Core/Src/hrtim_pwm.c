#include "hrtim_pwm.h"

static HRTIM_HandleTypeDef *pwm_hrtim = 0;
static float pwm_duty = HRTIM_PWM_DUTY_MIN;

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

static uint32_t pwm_adc_trigger_from_cmp1(uint32_t cmp1)
{
  uint32_t cmp3 = cmp1 + ((HRTIM_PWM_PERIOD_TICKS - cmp1) / 2UL);

  if (cmp3 >= (HRTIM_PWM_PERIOD_TICKS - 3UL)) {
    cmp3 = HRTIM_PWM_PERIOD_TICKS - 3UL;
  }
  if (cmp3 < 3UL) {
    cmp3 = 3UL;
  }

  return cmp3;
}

uint32_t HRTIM_PWM_DutyToTicks(float duty)
{
  const float clamped = pwm_clamp(duty, HRTIM_PWM_DUTY_MIN, HRTIM_PWM_DUTY_MAX);
  uint32_t ticks = (uint32_t)(clamped * (float)HRTIM_PWM_PERIOD_TICKS);

  if (ticks < 3UL) {
    ticks = 3UL;
  }
  if (ticks > (HRTIM_PWM_PERIOD_TICKS - 3UL)) {
    ticks = HRTIM_PWM_PERIOD_TICKS - 3UL;
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

void HRTIM_PWM_Start(void)
{
  if (pwm_hrtim == 0) {
    return;
  }

  HRTIM_PWM_StartCounter();
  (void)HAL_HRTIM_WaveformOutputStart(pwm_hrtim, HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);
}

void HRTIM_PWM_Stop(void)
{
  if (pwm_hrtim == 0) {
    return;
  }

  HRTIM_PWM_SetDuty(HRTIM_PWM_DUTY_MIN);
  (void)HAL_HRTIM_SoftwareUpdate(pwm_hrtim, HRTIM_TIMERUPDATE_D);
  (void)HAL_HRTIM_WaveformOutputStop(pwm_hrtim, HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);
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
