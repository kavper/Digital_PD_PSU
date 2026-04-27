#include "pid.h"
#include "app_power.h"

uint16_t adc_raw[PSU_ADC_CHANNEL_COUNT] = {0};

float current_setpoint_v = 0.0f;
float target_voltage = 0.0f;
float target_current_lim = I_TARGET_CURRENT;

static float clampf(float value, float min_value, float max_value)
{
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

void PID_Init(ADC_HandleTypeDef *hadc, HRTIM_HandleTypeDef *hhrtim)
{
  (void)hadc;
  (void)hhrtim;
  PSU_Init();
}

void PID_HandleInterrupt(void)
{
  PSU_ControlLoopFromAdc(psu_adc_dma_buffer);
}

void PID_AdcRestartDMA(void)
{
  /* ADC DMA is circular and HRTIM-triggered in the PSU layer. */
}

void PID_Reset(void)
{
  PSU_Stop();
  current_setpoint_v = 0.0f;
}

void PID_SetTargetVoltage(float val)
{
  target_voltage = clampf(val, 0.0f, 30.0f);
  PSU_SetVoltage(target_voltage);
}

void PID_SetTargetCurrent(float val)
{
  target_current_lim = clampf(val, 0.0f, 10.0f);
}

float PID_GetTargetVoltage(void)
{
  target_voltage = PSU_GetTargetVoltage();
  return target_voltage;
}

float PID_GetTargetCurrent(void)
{
  return target_current_lim;
}

float PID_GetVIn(void)
{
  return PSU_GetMeasuredInputVoltage();
}

float PID_GetVOut(void)
{
  return PSU_GetMeasuredVoltage();
}

float PID_GetIOut(void)
{
  return PSU_GetMeasuredCurrent();
}

float PID_GetVBoost(void)
{
  return PSU_GetMeasuredInputVoltage();
}

float PID_GetIIn(void)
{
  return 0.0f;
}

float PID_GetCurrentSetpoint(void)
{
  current_setpoint_v = PSU_GetSoftStartVoltage();
  return current_setpoint_v;
}

float PID_GetPWM(void)
{
  return PSU_GetDuty();
}

PID_ControlMode_t PID_GetControlMode(void)
{
  if (PSU_IsRunning() == 0U) {
    return PID_CONTROL_MODE_OFF;
  }

  return PID_CONTROL_MODE_CV;
}
