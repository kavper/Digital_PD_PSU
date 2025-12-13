#ifndef __PID_H
#define __PID_H

#include "main.h"

// --- KONFIGURACJA HRTIM ---
#define HRTIM_PERIOD 10880
#define PWM_MIN 10
#define PWM_MAX 10000

// --- NASTAWY ZASILACZA ---
#define V_TARGET_VOLTAGE 10.0f    // Docelowe napięcie wyjściowe
#define I_TARGET_CURRENT 1.5f     // Limit prądowy (CC) - bezpieczne 0.5A na start
#define SOFT_START_STEP  0.5f    // Krok narastania napięcia

// --- KALIBRACJA ADC ---
// Przeliczniki z wartości ADC (0-4095) na jednostki fizyczne (V, A)
#define COEFF_VOLTAGE 0.008682f
#define COEFF_CURRENT 0.002442f

// --- GETTERY (Dla GUI/Main) ---
float PID_GetCurrentSetpoint();
float PID_GetVIn();
float PID_GetVOut();
float PID_GetIOut();
float PID_GetVBoost();
float PID_GetIIn();
float PID_GetPWM();

// --- FUNKCJE STERUJĄCE ---
void PID_AdcRestartDMA(); // Funkcja do ręcznego restartu (dla stabilności)
void PID_Init(ADC_HandleTypeDef *hadc, HRTIM_HandleTypeDef *hhrtim);
void PID_HandleInterrupt();

#endif