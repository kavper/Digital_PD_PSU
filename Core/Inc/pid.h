#ifndef __PID_H
#define __PID_H

#include "main.h"

// --- KONFIGURACJA HRTIM ---
#define HRTIM_PERIOD 54400
#define PWM_MIN 2720  // 5% (Bezpieczny margines dla Bootstrapu)
#define PWM_MAX 51680 // 95%

// --- NASTAWY ZASILACZA ---
#define V_TARGET_VOLTAGE 12.0f // [V] Docelowe napięcie wyjściowe
#define SOFT_START_STEP 0.05f // [V] Krok narastania napięcia na pętlę

// --- ZABEZPIECZENIA ---
#define OVP_LIMIT 14.5f // [V] Over Voltage Protection (Wyłączenie)
#define OCP_LIMIT 10.0f // [A] Over Current Protection (Wyłączenie)
#define UVLO_LIMIT                                                             \
  15.0f // [V] Minimalne napięcie zasilania (V_BOOST) żeby w ogóle startować

// --- NASTAWY REGULATORA PID ---
// Wartości trzeba dobrać eksperymentalnie, te są "bezpieczne" na start
#define PID_KP 200.0f             // Wzmocnienie proporcjonalne
#define PID_KI 10.0f              // Wzmocnienie całkujące
#define PID_INTEGRAL_MAX 10000.0f // Limit członu całkującego (Anti-windup)

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

#endif /* __PID_H */