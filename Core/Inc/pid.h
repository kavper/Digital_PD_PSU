#ifndef __PID_H
#define __PID_H

#include "main.h"

typedef enum {
  PID_CONTROL_MODE_OFF = 0,
  PID_CONTROL_MODE_CV,
  PID_CONTROL_MODE_CC
} PID_ControlMode_t;

// --- KONFIGURACJA CZESTOTLIWOSCI ---
// Jedyne makro, ktore zmieniasz recznie.
#define PSU_SW_FREQ_KHZ 20UL

// Dzielnik probkowania ADC/PID:
// 1 = probkowanie co kazdy okres PWM
// 2 = co drugi okres PWM
// 5 = co piaty okres PWM
#define PSU_ADC_DIV 1UL

// Debug probe na pinie LED:
// 0 = wylaczone
// 1 = impuls na czas callbacku ADC
// 2 = impuls na czas wykonywania PID_HandleInterrupt()
#define PID_DEBUG_PIN_MODE 2U

#if (PSU_SW_FREQ_KHZ == 0UL)
#error "PSU_SW_FREQ_KHZ must be greater than 0"
#endif

#if (PSU_ADC_DIV == 0UL)
#error "PSU_ADC_DIV must be greater than 0"
#endif

enum {
  // Timer D HRTIM pracuje tutaj z 680000 kHz.
  HRTIM_PERIOD = (680000UL + (PSU_SW_FREQ_KHZ / 2UL)) / PSU_SW_FREQ_KHZ,
  // f_adc = f_pwm / PSU_ADC_DIV
  HRTIM_ADC_DIV = PSU_ADC_DIV,
  // W HAL postscaler=0 oznacza trigger co okres, wiec zapisujemy (div - 1).
  HRTIM_ADC_POSTSCALER = HRTIM_ADC_DIV - 1UL
};

// --- NASTAWY ZASILACZA ---
#define V_TARGET_VOLTAGE 0.0f     // Bezpieczniejszy start
#define I_TARGET_CURRENT 1.0f     

// --- KALIBRACJA ADC ---
#define COEFF_VOLTAGE 0.008682f
#define COEFF_CURRENT 0.002442f

// --- KOREKCJA KALIBRACJI POMIARU ---
// Offset napięcia wyłączony zgodnie z życzeniem.
#define VOUT_CAL_OFFSET_V  (0.0f)
#define IOUT_CAL_GAIN      (1.0f)
#define IOUT_CAL_OFFSET_A  (0.0f)

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
PID_ControlMode_t PID_GetControlMode(void);

#endif /* PID_H */
