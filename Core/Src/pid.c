#include "pid.h"
#include "main.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_hrtim.h"

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
float current_setpoint_v = 0.0f;       // Aktualne napięcie (narasta przy starcie)
float target_voltage     = V_TARGET_VOLTAGE;
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

// Funkcja restartująca ADC "na chama" (Gwarancja ciągłości działania)
void PID_AdcRestartDMA() {
    HAL_ADC_Start_DMA(adc_ptr, (uint32_t *)adc_raw, 5);
}

// ==========================================
// GETTERY (Dla GUI)
// ==========================================
float PID_GetVIn()    { return v_in; }
float PID_GetVOut()   { return v_out; }
float PID_GetIOut()   { return i_out; }
float PID_GetVBoost() { return v_boost; }
float PID_GetIIn()    { return i_in; }

// Zwraca aktualny setpoint napięcia
float PID_GetCurrentSetpoint() { 
    return current_setpoint_v; 
}

// Zwraca % wypełnienia PWM (0.0 - 1.0)
float PID_GetPWM() {
    if (!hrtim_ptr) return 0.0f;
    uint32_t cmp1 = hrtim_ptr->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR;
    return (float)cmp1 / (float)HRTIM_PERIOD;
}

// ==========================================
// CALLBACK ADC
// ==========================================
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        // 1. Oblicz PID
        PID_HandleInterrupt();
        
        // 2. Kopnij ADC żeby mierzyło dalej (Metoda brute-force)
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
    target_current_lim = I_TARGET_CURRENT;

    // Ustaw PWM na minimum
    hrtim_ptr->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = PWM_MIN;

    // Kalibracja ADC (Ważne w G4!)
    HAL_ADCEx_Calibration_Start(adc_ptr, ADC_SINGLE_ENDED);

    // Pierwszy start pomiaru
    PID_AdcRestartDMA();
}

// ==========================================
// GŁÓWNA PĘTLA REGULACJI
// ==========================================
void PID_HandleInterrupt() {
    // 1. Konwersja pomiarów
    v_in    = (float)adc_raw[0] * COEFF_VOLTAGE;
    v_out   = (float)adc_raw[1] * COEFF_VOLTAGE;
    i_out   = (float)adc_raw[2] * COEFF_CURRENT;
    v_boost = (float)adc_raw[3] * COEFF_VOLTAGE;
    i_in    = (float)adc_raw[4] * COEFF_CURRENT;

    current_setpoint_v = target_voltage;

    // 3. Feed Forward (Szacowanie wypełnienia)
    float pwm_ff = 0.0f;
    if (v_boost > 2.0f) {
        pwm_ff = (current_setpoint_v / v_boost) * (float)HRTIM_PERIOD;
    }

    // 4. Obliczenia Regulatorów
    
    // --- Tor Napięciowy (CV) ---
    float err_v = current_setpoint_v - v_out;
    float out_cv = pwm_ff + (err_v * CV_KP) + cv_integral + (err_v * CV_KI);

    // --- Tor Prądowy (CC) ---
    float err_i = target_current_lim - i_out;
    float out_cc = pwm_ff + (err_i * CC_KP) + cc_integral + (err_i * CC_KI);

    // 5. Wybór trybu (Bump-less Transfer)
    // To jest klucz do stabilnego zwarcia!
    float final_pwm = 0.0f;

    // --- DODATKOWE ZABEZPIECZENIE (SHORT CIRCUIT PROTECTION) ---
    // Jeśli napięcie wyjściowe spadło drastycznie (np. poniżej 1V), a prąd jest duży,
    // to mamy twarde zwarcie. Natychmiast zerujemy całki i tniemy PWM.
    // To ratuje zasilacz PD przed odcięciem.
    
    if (v_out < 1.0f && i_out > (target_current_lim * 0.8f)) {
        // Wykryto twarde zwarcie!
        
        // 1. Resetujemy całkę napięciową (żeby nie pompowała w górę)
        cv_integral = 0.0f; 
        
        // 2. Ustawiamy całkę prądową nisko, żeby startowała od zera
        cc_integral = 0.0f;
        
        // 3. Wymuszamy minimalny PWM w tym cyklu
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

    // 6. Saturacja (Anti-windup sztywny)
    cv_integral = clamp(cv_integral, -PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);
    cc_integral = clamp(cc_integral, -PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);

    // 7. Zapis do PWM
    final_pwm = clamp(final_pwm, (float)PWM_MIN, (float)PWM_MAX);
    uint32_t u32_pwm = (uint32_t)final_pwm;

    hrtim_ptr->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = u32_pwm;

    // 8. Ustawienie Triggera ADC w ŚRODKU IMPULSU
    // Nawet przy metodzie restartowanej, to poprawia jakość odczytu!
    uint32_t cmp3 = u32_pwm / 2;
    if (cmp3 < 200) cmp3 = 200; // Minimalny czas dla ADC
    hrtim_ptr->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP3xR = cmp3;
}



void PID_SetTargetVoltage(float val) {
    target_voltage = val;
}

void PID_SetTargetCurrent(float val) {
    target_current_lim = val;
}

float PID_GetTargetVoltage(void)
{
    return target_voltage;
}

float PID_GetTargetCurrent(void)
{
    return target_current_lim;
}