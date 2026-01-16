#ifndef __PID_H
#define __PID_H

#include "main.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_hrtim.h"

// --- KONFIGURACJA HRTIM ---
#define HRTIM_PERIOD 10880

// Minimalny PWM.
// Jeśli zostawisz >0, to zasilacz "zawsze trochę pompuje" nawet przy 0V set.
#define PWM_MIN 0
#define PWM_MAX 10880

// --- NASTAWY STARTOWE (SAFETY) ---
#define V_TARGET_VOLTAGE 0.0f
#define I_TARGET_CURRENT 1.0f

// --- KALIBRACJA ADC ---
#define COEFF_VOLTAGE 0.008682f
#define COEFF_CURRENT 0.002442f

// --- GETTERY ---
float PID_GetCurrentSetpoint(void);
float PID_GetVIn(void);
float PID_GetVOut(void);
float PID_GetIOut(void);
float PID_GetVBoost(void);
float PID_GetIIn(void);
float PID_GetPWM(void);

float PID_GetTargetVoltage(void);
float PID_GetTargetCurrent(void);

// --- FUNKCJE ---
void PID_AdcRestartDMA(void);
void PID_Init(ADC_HandleTypeDef *hadc, HRTIM_HandleTypeDef *hhrtim);
void PID_HandleInterrupt(void);

void PID_SetTargetVoltage(float val);
void PID_SetTargetCurrent(float val);

/* Reset stanu regulatorów + PWM na minimum (bez zmiany algorytmu). */
void PID_Reset(void);

#endif