#ifndef POWER_MEASUREMENT_H
#define POWER_MEASUREMENT_H

#include <stdint.h>

#define POWER_ADC_VREF_V      (3.0f)
#define POWER_ADC_FULL_SCALE  (4095.0f)
#define POWER_VOUT_GAIN       (0.08438f)
#define POWER_CURRENT_GAIN_VA (0.300f)
#define POWER_ADC_TO_V        (POWER_ADC_VREF_V / POWER_ADC_FULL_SCALE)
#define POWER_ADC_TO_VOUT     (POWER_ADC_TO_V / POWER_VOUT_GAIN)
#define POWER_ADC_TO_CURRENT  (POWER_ADC_TO_V / POWER_CURRENT_GAIN_VA)

typedef struct
{
  float vout_adc_v;
  float vout_v;
  float iout_adc_v;
  float iout_a;
  float vboost_adc_v;
  float vboost_v;
} PowerMeasurements_t;

float PowerMeasurement_RawToAdcVoltage(uint16_t raw);
float PowerMeasurement_AdcToVout(float adc_voltage_v);
float PowerMeasurement_RawToVout(uint16_t raw);
float PowerMeasurement_RawToCurrent(uint16_t raw);
void PowerMeasurement_Update(PowerMeasurements_t *measurements,
                             uint16_t raw_vout,
                             uint16_t raw_iout,
                             uint16_t raw_vboost);

#endif /* POWER_MEASUREMENT_H */
