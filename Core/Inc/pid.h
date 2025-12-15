#ifndef __PID_H
#define __PID_H

#include "main.h"

// --- KONFIGURACJA HRTIM ---
#define HRTIM_PERIOD 10880

// !!! NAPRAWA 1: Zmień 100 na 0. 
// Inaczej zasilacz zawsze będzie pompował trochę napięcia!
#define PWM_MIN 100         
#define PWM_MAX 10880

// --- NASTAWY ZASILACZA ---
#define V_TARGET_VOLTAGE 0.0f     // Bezpieczniejszy start
#define I_TARGET_CURRENT 1.0f     
#define SOFT_START_STEP  0.5f     // Wolniejszy soft-start jest bezpieczniejszy

// --- KALIBRACJA ADC ---
#define COEFF_VOLTAGE 0.008682f
#define COEFF_CURRENT 0.002442f

// --- GETTERY ---
float PID_GetCurrentSetpoint();
float PID_GetVIn();
float PID_GetVOut();
float PID_GetIOut();
float PID_GetVBoost();
float PID_GetIIn();
float PID_GetPWM();

// --- FUNKCJE STERUJĄCE ---
void PID_AdcRestartDMA();
void PID_Init(ADC_HandleTypeDef *hadc, HRTIM_HandleTypeDef *hhrtim);
void PID_HandleInterrupt();

void PID_SetTargetVoltage(float val);
void PID_SetTargetCurrent(float val);

float PID_GetTargetVoltage(void);
float PID_GetTargetCurrent(void);

#endif