#include "pid.h"
#include "main.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_hrtim.h"

// --- POMIARY ---
static float v_in = 0.0f;
static float v_out = 0.0f;
static float i_out = 0.0f;
static float v_boost = 0.0f;
static float i_in = 0.0f;

uint16_t adc_raw[5] = {0};

// --- STANY STEROWANIA ---
float pwm_ff = 0.0f;
float pwm_pid = 0.0f;      // tutaj będzie użyte jako "delta" z aktywnego regulatora
float pwm_total = 0.0f;
float error = 0.0f;

// current_setpoint = referencja napięciowa (V) – tak jak u Ciebie
float current_setpoint   = 0.0f;           // zadane napięcie w trakcie soft-startu
float current_setpoint_i = I_TARGET_CURRENT; // maksymalny prąd CC (2 A domyślnie)

// dwa integratory – osobno dla CV i CC
static float cv_integral = 0.0f;
static float cc_integral = 0.0f;


HRTIM_HandleTypeDef *hrtim = NULL;
ADC_HandleTypeDef   *adc   = NULL;

// --- DODATKOWE NASTAWY PI (lokalne tylko dla tego pliku) ---
// Pętla napięciowa
#define CV_KP  150.0f
#define CV_KI  5.0f

// Pętla prądowa (ostrzejsza)
#define CC_KP  80.0f
#define CC_KI  8.0f

// --- GETTERY ---

float PID_GetCurrentSetpoint() { 
    return current_setpoint;   // w praktyce "Vref" po soft-starcie
}

float PID_GetVIn()    { return v_in; }
float PID_GetVOut()   { return v_out; }
float PID_GetIOut()   { return i_out; }
float PID_GetVBoost() { return v_boost; }
float PID_GetIIn()    { return i_in; }

float PID_GetPWM() {
    if (!hrtim) return 0.0f;
    return (float)hrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR /
           (float)HRTIM_PERIOD;
}

// --- UTIL ---

static float clamp(float v, float mn, float mx) {
    if (v < mn) return mn;
    if (v > mx) return mx;
    return v;
}

void PID_AdcRestartDMA() {
    HAL_ADC_Start_DMA(adc, (uint32_t *)adc_raw, 5);
}

// --- CALLBACK ADC ---

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        PID_HandleInterrupt();
        PID_AdcRestartDMA();
    }
}

// --- INIT ---

void PID_Init(ADC_HandleTypeDef *hadc, HRTIM_HandleTypeDef *hhrtim) {
    adc = hadc;
    hrtim = hhrtim;

    v_in = v_out = i_out = v_boost = i_in = 0.0f;
    pwm_ff = pwm_pid = pwm_total = 0.0f;
    error = 0.0f;

    current_setpoint   = 0.0f;           // startujemy od zera (soft start)
    current_setpoint_i = I_TARGET_CURRENT;

    cv_integral = 0.0f;
    cc_integral = 0.0f;


    HAL_ADC_Start_DMA(adc, (uint32_t *)adc_raw, 5);
}

// --- GŁÓWNY ALGORYTM CC/CV ---
// Wywoływany 50 kHz z przerwania ADC

void PID_HandleInterrupt() {
    // --- ODCZYT I PRZESKALOWANIE ADC ---
    v_in    = (float)adc_raw[0] * COEFF_VOLTAGE;
    v_out   = (float)adc_raw[1] * COEFF_VOLTAGE;
    i_out   = (float)adc_raw[2] * COEFF_CURRENT;
    v_boost = (float)adc_raw[3] * COEFF_VOLTAGE;
    i_in    = (float)adc_raw[4] * COEFF_CURRENT;

    // --- UVLO / BRAK ZASILANIA NA BOOST ---
    if (v_boost <= UVLO_LIMIT) {
        current_setpoint = 0.0f;
        cv_integral = 0.0f;
        cc_integral = 0.0f;

        if (hrtim) {
            hrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = PWM_MIN;
        }
        return;
    }


    // --- TWARDY LIMIT PRĄDOWY ---
    if (i_out > I_LIMIT_MAX) {
        if (hrtim) {
            hrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = PWM_MIN;
        }
        // Czyścimy szybki integrator prądowy, żeby po zejściu z zwarcia się nie wlókł
        cc_integral = 0.0f;
        return;
    }

    // --- SOFT-START NAPIĘCIA (SZYBKI, JAK U CIEBIE) ---
    if (current_setpoint < V_TARGET_VOLTAGE) {
        current_setpoint += SOFT_START_STEP;  // 0.05 V / próbkę -> ~2 ms do 5 V
        if (current_setpoint > V_TARGET_VOLTAGE) {
            current_setpoint = V_TARGET_VOLTAGE;
        }
    }

    // --- FEED-FORWARD ---
    // Buck: Vout ≈ D * Vboost  => D_ff = Vref / Vboost
    float d_ff = 0.0f;
    if (v_boost > 1.0f) {
        d_ff = current_setpoint / v_boost;
    }
    d_ff = clamp(d_ff, (float)PWM_MIN / (float)HRTIM_PERIOD,
                       (float)PWM_MAX / (float)HRTIM_PERIOD);
    pwm_ff = d_ff * (float)HRTIM_PERIOD;

    // =====================================================
    //  PĘTLA NAPIĘCIOWA (CV) – PI na Vout
    // =====================================================
    float v_error = current_setpoint - v_out;

    cv_integral += (v_error * CV_KI);
    cv_integral = clamp(cv_integral, -PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);

    float pwm_cv = pwm_ff + (v_error * CV_KP) + cv_integral;

    // =====================================================
    //  PĘTLA PRĄDOWA (CC) – PI na Iout (limit)
    // =====================================================
    float i_error = current_setpoint_i - i_out;

    cc_integral += (i_error * CC_KI);
    cc_integral = clamp(cc_integral, -PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);

    float pwm_cc = pwm_ff + (i_error * CC_KP) + cc_integral;

    // =====================================================
    //  WYBÓR PĘTLI: MIN(pwm_cv, pwm_cc)
    //  - dopóki prąd < limitu: pwm_cc zwykle >= pwm_cv -> steruje napięcie (CV)
    //  - gdy prąd chce wyjść ponad limit: pwm_cc spada < pwm_cv -> przejście w CC
    // =====================================================
    float pwm_raw = (pwm_cc < pwm_cv) ? pwm_cc : pwm_cv;

    // --- OGRANICZENIE I LEKKI ANTY-WINDUP ---
    float pwm_clamped = clamp(pwm_raw, (float)PWM_MIN, (float)PWM_MAX);

    if (pwm_raw != pwm_clamped) {
        // Jeżeli saturacja i regulator dalej "pcha" w tą stronę – upuszczamy trochę integratora
        if ((pwm_raw > PWM_MAX && v_error > 0.0f) ||
            (pwm_raw < PWM_MIN && v_error < 0.0f)) {
            cv_integral *= 0.9f;
        }
        if ((pwm_raw > PWM_MAX && i_error > 0.0f) ||
            (pwm_raw < PWM_MIN && i_error < 0.0f)) {
            cc_integral *= 0.9f;
        }
    }

    pwm_total = pwm_clamped;
    pwm_pid   = pwm_total - pwm_ff;   // dla debuggera, żebyś widział ile robi PI

    uint32_t final_compare = (uint32_t)pwm_clamped;

    if (hrtim) {
        hrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = final_compare;
    }
}
