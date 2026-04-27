#ifndef APP_POWER_H
#define APP_POWER_H

#include <stdint.h>

#define PSU_ADC_CHANNEL_COUNT (3U)

enum {
  PSU_ADC_INDEX_VOUT = 0,
  PSU_ADC_INDEX_IOUT = 1,
  PSU_ADC_INDEX_VBOOST = 2
};

extern uint16_t psu_adc_dma_buffer[PSU_ADC_CHANNEL_COUNT];

void PSU_Init(void);
void PSU_Start(void);
void PSU_Stop(void);
void PSU_SetVoltage(float voltage_v);
void PSU_ControlLoopFromAdc(const uint16_t adc_samples[PSU_ADC_CHANNEL_COUNT]);
void PSU_AdcDmaIrqHandler(void);
void PSU_Service(void);
float PSU_GetMeasuredVoltage(void);
float PSU_GetMeasuredCurrent(void);
float PSU_GetMeasuredInputVoltage(void);
float PSU_GetDuty(void);
float PSU_GetTargetVoltage(void);
float PSU_GetSoftStartVoltage(void);
uint8_t PSU_IsRunning(void);

#endif /* APP_POWER_H */
