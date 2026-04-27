#ifndef HRTIM_PWM_H
#define HRTIM_PWM_H

#include "main.h"

#define HRTIM_PWM_FSW_HZ        (200000UL)
#define HRTIM_PWM_TIMER_CLOCK_HZ (5440000000ULL)
#define HRTIM_PWM_PERIOD_TICKS  (27200UL)
/*
 * Allow true low-voltage operation. The PSU layer disables the power stage
 * for a 0 V setpoint, while nonzero low setpoints may use very small duty.
 */
#define HRTIM_PWM_DUTY_MIN      (0.0f)
#define HRTIM_PWM_DUTY_MAX      (0.80f)

#ifndef HRTIM_PWM_DEBUG_ISR_GPIO
#define HRTIM_PWM_DEBUG_ISR_GPIO 0
#endif

#if HRTIM_PWM_DEBUG_ISR_GPIO
#define DEBUG_ADC_ISR_HIGH() \
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET)
#define DEBUG_ADC_ISR_LOW() \
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET)
#else
#define DEBUG_ADC_ISR_HIGH() do { } while (0)
#define DEBUG_ADC_ISR_LOW()  do { } while (0)
#endif

void HRTIM_PWM_Init(HRTIM_HandleTypeDef *hhrtim);
void HRTIM_PWM_StartCounter(void);
void HRTIM_PWM_StopCounter(void);
void HRTIM_PWM_EnableOutputs(void);
void HRTIM_PWM_DisableOutputs(void);
void HRTIM_PWM_ForceUpdate(void);
void HRTIM_PWM_Start(void);
void HRTIM_PWM_Stop(void);
void HRTIM_PWM_SetDuty(float duty);
float HRTIM_PWM_GetDuty(void);
uint32_t HRTIM_PWM_DutyToTicks(float duty);
uint32_t HRTIM_PWM_GetPeriodTicks(void);

#endif /* HRTIM_PWM_H */
