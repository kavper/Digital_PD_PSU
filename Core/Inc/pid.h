#ifndef __PID_H
#define __PID_H

#include "main.h"

// --- KONFIGURACJA HRTIM ---
#define HRTIM_PERIOD 34000

// 0 PWM przy wyjściu OFF / 0V.
#define PWM_MIN 0
#define PWM_MAX 34000

// --- NASTAWY ZASILACZA ---
#define V_TARGET_VOLTAGE 0.0f     // Bezpieczniejszy start
#define I_TARGET_CURRENT 1.0f     
#define SOFT_START_STEP  0.005f     // Wolniejszy soft-start jest bezpieczniejszy

// --- KALIBRACJA ADC ---
#define COEFF_VOLTAGE 0.008682f
#define COEFF_CURRENT 0.002442f

// --- KOREKCJA KALIBRACJI POMIARU ---
// Offset napięcia wyłączony zgodnie z życzeniem.
#define VOUT_CAL_OFFSET_V  (0.0f)
// Iout: błąd liniowy (1.00A -> 0.95A, 4.00A -> 3.80A) => gain 0.95.
#define IOUT_CAL_GAIN      (0.95f)
#define IOUT_CAL_OFFSET_A  (0.0f)

// --- CZĘSTOTLIWOŚĆ PĘTLI REGULACJI ---
// Dla f_sw=20kHz i ADC postscaler=4 => ~4kHz.
#define CTRL_LOOP_HZ       (4000.0f)


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
