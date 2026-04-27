#ifndef __PID_H
#define __PID_H

#include "main.h"

typedef enum {
  PID_CONTROL_MODE_OFF = 0,
  PID_CONTROL_MODE_CV,
  PID_CONTROL_MODE_CC
} PID_ControlMode_t;

// --- KONFIGURACJA CZESTOTLIWOSCI ---
#define PSU_SW_FREQ_KHZ 200UL

// Dzielnik probkowania ADC/PID:
// 4 = PWM 200 kHz, petla ADC/CV 50 kHz.
// Proba pracy przez HAL/DMA przy 200 kHz zagladzala while(1) i OLED po ON.
#define PSU_ADC_DIV 4UL

#if (PSU_SW_FREQ_KHZ == 0UL)
#error "PSU_SW_FREQ_KHZ must be greater than 0"
#endif

#if (PSU_ADC_DIV == 0UL)
#error "PSU_ADC_DIV must be greater than 0"
#endif

enum {
  // Timer D HRTIM: 170 MHz * 32 = 5.44 GHz, PER=27200 -> 200 kHz.
  HRTIM_PERIOD = 27200UL,
  // f_adc = f_pwm / PSU_ADC_DIV
  HRTIM_ADC_DIV = PSU_ADC_DIV,
  // W HAL postscaler=0 oznacza trigger co okres, wiec zapisujemy (div - 1).
  HRTIM_ADC_POSTSCALER = HRTIM_ADC_DIV - 1UL
};

// --- NASTAWY ZASILACZA ---
#define V_TARGET_VOLTAGE 0.0f     // Bezpieczniejszy start
#define I_TARGET_CURRENT 1.0f     

/* Legacy alias; nowy bufor DMA ma 3 kanaly w app_power.c. */
extern uint16_t adc_raw[3];

/* Zadane wartości (dla GUI / logiki) */
extern float current_setpoint_v;   /* aktualny setpoint po rampie */
extern float target_voltage;       /* target z GUI */
extern float target_current_lim;   /* limit prądu z GUI */

/* Init / ISR */
void PID_Init(ADC_HandleTypeDef *hadc, HRTIM_HandleTypeDef *hhrtim);
void PID_HandleInterrupt(void);
void PID_AdcRestartDMA(void);

/* Reset bezpieczeństwa (integratory + PWM min + setpoint=0) */
void PID_Reset(void);

/* Settery / Gettery */
void  PID_SetTargetVoltage(float val);
void  PID_SetTargetCurrent(float val);
float PID_GetTargetVoltage(void);
float PID_GetTargetCurrent(void);

float PID_GetVIn(void);
float PID_GetVOut(void);
float PID_GetIOut(void);
float PID_GetVBoost(void);
float PID_GetIIn(void);

float PID_GetCurrentSetpoint(void);
float PID_GetPWM(void);
PID_ControlMode_t PID_GetControlMode(void);

#endif /* PID_H */
