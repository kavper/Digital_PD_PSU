#include "psu_gui_api.h"
#include "app_power.h"
#include "psu_config.h"

static float gui_target_current_a = PSU_DEFAULT_CURRENT_LIMIT_A;

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

void PSU_GuiInit(void)
{
  PSU_Init();
}

void PSU_GuiReset(void)
{
  PSU_Stop();
}

void PSU_GuiSetTargetVoltage(float voltage_v)
{
  PSU_SetVoltage(clampf(voltage_v, 0.0f, 30.0f));
}

void PSU_GuiSetTargetCurrent(float current_a)
{
  /*
   * Current control is intentionally not closed yet. Keep this value for the
   * existing GUI/debug display so the interface behaves as before.
   */
  gui_target_current_a = clampf(current_a, 0.0f, 10.0f);
}

float PSU_GuiGetTargetVoltage(void)
{
  return PSU_GetTargetVoltage();
}

float PSU_GuiGetTargetCurrent(void)
{
  return gui_target_current_a;
}

float PSU_GuiGetInputVoltage(void)
{
  return PSU_GetMeasuredInputVoltage();
}

float PSU_GuiGetOutputVoltage(void)
{
  return PSU_GetMeasuredVoltage();
}

float PSU_GuiGetOutputCurrent(void)
{
  return PSU_GetMeasuredCurrent();
}

float PSU_GuiGetBoostVoltage(void)
{
  return PSU_GetMeasuredInputVoltage();
}

float PSU_GuiGetInputCurrent(void)
{
  return 0.0f;
}

float PSU_GuiGetSlewedSetpointVoltage(void)
{
  return PSU_GetSoftStartVoltage();
}

float PSU_GuiGetDuty(void)
{
  return PSU_GetDuty();
}

PSU_GuiControlMode_t PSU_GuiGetControlMode(void)
{
  if (PSU_IsRunning() == 0U) {
    return PSU_GUI_CONTROL_MODE_OFF;
  }

  return PSU_GUI_CONTROL_MODE_CV;
}
