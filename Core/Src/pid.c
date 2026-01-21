#include "pid.h"
#include "main.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_hrtim.h"
#include <stdbool.h>

// ==========================================
// NASTAWY REGULATORA
// ==========================================

// Parametry PID (Dobierz pod swoje cewki, te są bezpieczne na start)
#define CV_KP  100.0f
#define CV_KI  10.0f

#define CC_KP  100.0f
#define CC_KI  10.0f

// Limit całki (zabezpieczenie przed nasyceniem)
#define PID_INTEGRAL_MAX  ((float)HRTIM_PERIOD * 0.9f)

// ==========================================
// ZMIENNE
// ==========================================

// Bufor DMA
uint16_t adc_raw[5] = {0};

// Pomiary fizyczne
static float v_in = 0.0f;
static float v_out = 0.0f;
static float i_out = 0.0f;
static float v_boost = 0.0f;
static float i_in = 0.0f;

// Zadane wartości
float current_setpoint_v = 0.0f;   // teraz to tylko "mirror" target_voltage
float target_voltage     = 0.0f;   // GUI ustawia
float target_current_lim = I_TARGET_CURRENT;

// Integratory
static float cv_integral = 0.0f;
static float cc_integral = 0.0f;

// Uchwyty
HRTIM_HandleTypeDef *hrtim_ptr = NULL;
ADC_HandleTypeDef   *adc_ptr   = NULL;

// ==========================================
// FUNKCJE POMOCNICZE
// ==========================================

static inline float clamp(float v, float mn, float mx) {
    if (v < mn) return mn;
    if (v > mx) return mx;
    return v;
}

static inline bool hw_output_enabled(void) {
    return (HAL_GPIO_ReadPin(DRV_EN_GPIO_Port, DRV_EN_Pin) == GPIO_PIN_SET);
}

// Funkcja restartująca ADC
void PID_AdcRestartDMA(void) {
    if (!adc_ptr) return;
    (void)HAL_ADC_Start_DMA(adc_ptr, (uint32_t *)adc_raw, 5);
}

// ==========================================
// GETTERY (Dla GUI)
// ==========================================
float PID_GetVIn(void)    { return v_in; }
float PID_GetVOut(void)   { return v_out; }
float PID_GetIOut(void)   { return i_out; }
float PID_GetVBoost(void) { return v_boost; }
float PID_GetIIn(void)    { return i_in; }

float PID_GetCurrentSetpoint(void) {
    return current_setpoint_v;
}

float PID_GetPWM(void) {
    if (!hrtim_ptr) return 0.0f;
    uint32_t cmp1 = hrtim_ptr->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR;
    return (float)cmp1 / (float)HRTIM_PERIOD;
}

// ==========================================
// RESET (bezpieczny)
// ==========================================
void PID_Reset(void) {
    cv_integral = 0.0f;
    cc_integral = 0.0f;
    current_setpoint_v = 0.0f;

    if (hrtim_ptr) {
        hrtim_ptr->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = PWM_MIN;

        uint32_t cmp3 = ((uint32_t)PWM_MIN) / 2u;
        if (cmp3 < 200u) cmp3 = 200u;
        hrtim_ptr->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP3xR = cmp3;
    }
}

// ==========================================
// CALLBACK ADC
// ==========================================
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        // 1. Oblicz PID
        PID_HandleInterrupt();

        // 2. Restart ADC
        PID_AdcRestartDMA();
    }
}

// ==========================================
// INIT
// ==========================================
void PID_Init(ADC_HandleTypeDef *hadc, HRTIM_HandleTypeDef *hhrtim) {
    adc_ptr = hadc;
    hrtim_ptr = hhrtim;

    // Reset zmiennych
    v_in = v_out = i_out = v_boost = i_in = 0.0f;
    cv_integral = cc_integral = 0.0f;
    current_setpoint_v = 0.0f;

    // Bezpieczny start: bez napięcia
    target_voltage = 0.0f;
    target_current_lim = I_TARGET_CURRENT;

    // Ustaw PWM na minimum
    if (hrtim_ptr) {
        hrtim_ptr->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = PWM_MIN;

        uint32_t cmp3 = ((uint32_t)PWM_MIN) / 2u;
        if (cmp3 < 200u) cmp3 = 200u;
        hrtim_ptr->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP3xR = cmp3;
    }

    // Kalibracja ADC (Ważne w G4!)
    if (adc_ptr) {
        (void)HAL_ADCEx_Calibration_Start(adc_ptr, ADC_SINGLE_ENDED);
    }

    // Pierwszy start pomiaru
    PID_AdcRestartDMA();
}

// ==========================================
// GŁÓWNA PĘTLA REGULACJI
// ==========================================
void PID_HandleInterrupt(void) {
    // 1. Konwersja pomiarów
    v_in    = (float)adc_raw[0] * COEFF_VOLTAGE;
    v_out   = (float)adc_raw[1] * COEFF_VOLTAGE;
    i_out   = (float)adc_raw[2] * COEFF_CURRENT;
    v_boost = (float)adc_raw[3] * COEFF_VOLTAGE;
    i_in    = (float)adc_raw[4] * COEFF_CURRENT;

    // 2. Bezpieczeństwo: gdy wyjście wyłączone lub target=0, nie całkuj i trzymaj PWM na minimum
    if (!hw_output_enabled() || (target_voltage <= 0.0f)) {
        cv_integral = 0.0f;
        cc_integral = 0.0f;
        current_setpoint_v = 0.0f;

        // PWM minimalne
        if (hrtim_ptr) {
            hrtim_ptr->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = PWM_MIN;

            uint32_t cmp3 = ((uint32_t)PWM_MIN) / 2u;
            if (cmp3 < 200u) cmp3 = 200u;
            hrtim_ptr->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP3xR = cmp3;
        }
        return;
    }

    // 3. BRAK SOFT-STARTU: od razu bierzemy target jako setpoint
    current_setpoint_v = target_voltage;

    // 4. Feed Forward (Szacowanie wypełnienia)
    float pwm_ff = 0.0f;
    if (v_boost > 2.0f) {
        pwm_ff = (current_setpoint_v / v_boost) * (float)HRTIM_PERIOD;
    }

    // 5. Obliczenia Regulatorów

    // --- Tor Napięciowy (CV) ---
    float err_v = current_setpoint_v - v_out;
    float out_cv = pwm_ff + (err_v * CV_KP) + cv_integral + (err_v * CV_KI);

    // --- Tor Prądowy (CC) ---
    float err_i = target_current_lim - i_out;
    float out_cc = pwm_ff + (err_i * CC_KP) + cc_integral + (err_i * CC_KI);

    // 6. Wybór trybu (Bump-less Transfer)
    float final_pwm = 0.0f;

    // --- DODATKOWE ZABEZPIECZENIE (SHORT CIRCUIT PROTECTION) ---
    if (v_out < 1.0f && i_out > (target_current_lim * 0.8f)) {
        // Wykryto twarde zwarcie!
        cv_integral = 0.0f;
        cc_integral = 0.0f;
        out_cc = (float)PWM_MIN;
    }

    if (out_cc < out_cv) {
        // -> Tryb CC (Ograniczenie prądu)
        final_pwm = out_cc;

        // Aktualizujemy integrator CC
        cc_integral += (err_i * CC_KI);

        // Oszukujemy integrator CV (żeby był gotowy do powrotu)
        cv_integral = final_pwm - pwm_ff - (err_v * CV_KP);

    } else {
        // -> Tryb CV (Normalna praca)
        final_pwm = out_cv;

        // Aktualizujemy integrator CV
        cv_integral += (err_v * CV_KI);

        // Oszukujemy integrator CC (żeby czekał tuż nad limitem)
        cc_integral = final_pwm - pwm_ff - (err_i * CC_KP);
    }

    // 7. Saturacja (Anti-windup sztywny)
    cv_integral = clamp(cv_integral, -PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);
    cc_integral = clamp(cc_integral, -PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);

    // 8. Zapis do PWM
    final_pwm = clamp(final_pwm, (float)PWM_MIN, (float)PWM_MAX);
    uint32_t u32_pwm = (uint32_t)final_pwm;

    if (hrtim_ptr) {
        hrtim_ptr->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = u32_pwm;

        // 9. Trigger ADC w środku impulsu
        uint32_t cmp3 = u32_pwm / 2u;
        if (cmp3 < 200u) cmp3 = 200u;
        hrtim_ptr->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP3xR = cmp3;
    }
}

void PID_SetTargetVoltage(float val) {
    target_voltage = val;
}

void PID_SetTargetCurrent(float val) {
    target_current_lim = val;
}

float PID_GetTargetVoltage(void) {
    return target_voltage;
}

float PID_GetTargetCurrent(void) {
    return target_current_lim;
}