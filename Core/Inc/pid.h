#ifndef __PID_H
#define __PID_H

#include "main.h"

// --- KONFIGURACJA HRTIM ---
#define HRTIM_PERIOD 10880

// !!! NAPRAWA 1: Zmień 100 na 0. 
// Inaczej zasilacz zawsze będzie pompował trochę napięcia!
#define PWM_MIN 0         
#define PWM_MAX 10880

// --- NASTAWY ZASILACZA ---
#define V_TARGET_VOLTAGE 0.0f     // Bezpieczniejszy start
#define I_TARGET_CURRENT 1.0f     
#define SOFT_START_STEP  0.005f     // Wolniejszy soft-start jest bezpieczniejszy

// --- KALIBRACJA ADC ---
#define COEFF_VOLTAGE 0.008682f
#define COEFF_CURRENT 0.002442f


/* Bufor DMA ADC (5 kanałów) */
extern uint16_t adc_raw[5];

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

#endif /* PID_H */