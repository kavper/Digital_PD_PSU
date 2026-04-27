#include "power_measurement.h"

float PowerMeasurement_RawToAdcVoltage(uint16_t raw)
{
  return (float)raw * POWER_ADC_TO_V;
}

float PowerMeasurement_AdcToVout(float adc_voltage_v)
{
  return adc_voltage_v / POWER_VOUT_GAIN;
}

float PowerMeasurement_RawToVout(uint16_t raw)
{
  return (float)raw * POWER_ADC_TO_VOUT;
}

float PowerMeasurement_RawToCurrent(uint16_t raw)
{
  return (float)raw * POWER_ADC_TO_CURRENT;
}

void PowerMeasurement_Update(PowerMeasurements_t *measurements,
                             uint16_t raw_vout,
                             uint16_t raw_iout,
                             uint16_t raw_vboost)
{
  if (measurements == 0) {
    return;
  }

  measurements->vout_adc_v = (float)raw_vout * POWER_ADC_TO_V;
  measurements->vout_v = (float)raw_vout * POWER_ADC_TO_VOUT;
  measurements->iout_adc_v = (float)raw_iout * POWER_ADC_TO_V;
  measurements->iout_a = (float)raw_iout * POWER_ADC_TO_CURRENT;

  measurements->vboost_adc_v = (float)raw_vboost * POWER_ADC_TO_V;
  measurements->vboost_v = (float)raw_vboost * POWER_ADC_TO_VOUT;
}
