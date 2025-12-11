#ifndef __PID_H
#define __PID_H

#include "main.h"

// --- KONFIGURACJA HRTIM ---
#define HRTIM_PERIOD 10880
#define PWM_MIN 100
#define PWM_MAX 9000

// --- NASTAWY ZASILACZA ---
#define V_TARGET_VOLTAGE 10.0f
#define SOFT_START_STEP 0.05f

// --- TRYB CC ---
#define I_TARGET_CURRENT   0.5f
#define I_LIMIT_MAX        1.0f

// --- ZABEZPIECZENIA ---
#define OVP_LIMIT 14.5f
#define OCP_LIMIT 10.0f
#define UVLO_LIMIT 15.0f

// --- NASTAWY PID ---
#define PID_KP 2.0f
#define PID_KI 10.0f
#define PID_INTEGRAL_MAX 10000.0f

// --- KALIBRACJA ADC ---
#define COEFF_VOLTAGE 0.008682f
#define COEFF_CURRENT 0.002442f

float PID_GetCurrentSetpoint();
float PID_GetVIn();
float PID_GetVOut();
float PID_GetIOut();
float PID_GetVBoost();
float PID_GetIIn();
float PID_GetPWM();

void PID_AdcRestartDMA();
void PID_Init(ADC_HandleTypeDef *hadc, HRTIM_HandleTypeDef *hhrtim);
void PID_HandleInterrupt();

#endif
