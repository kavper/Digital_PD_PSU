#include "pid.h"
#include "main.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_hrtim.h"
#include <math.h>
#include <stdbool.h>

// ==========================================
// NASTAWY REGULATORA
// ==========================================

// Sterowanie pracuje w callbacku ADC.
#define CTRL_FS_HZ \
  ((1000.0f * (float)PSU_SW_FREQ_KHZ) / (float)(HRTIM_ADC_POSTSCALER + 1UL))
#define CTRL_TS_S  (1.0f / CTRL_FS_HZ)

// 0 PWM przy wyjsciu OFF / 0V.
#define PWM_MIN            10U
#define PWM_HEADROOM_TICKS (HRTIM_PERIOD / 34U)
#define PWM_MAX            (HRTIM_PERIOD - PWM_HEADROOM_TICKS)

// Strojenie regulatorów (jednostka wyjścia: "ticks" PWM).
// CV = stabilizacja napięcia, CC = stabilizacja prądu.
#define CV_KP (360.0f)
#define CV_KI (1200.0f)

#define CC_KP (1100.0f)
#define CC_KI (2600.0f)

// Back-calculation anti-windup.
#define PI_AW_GAIN (0.25f)

// Soft-start i rampy nastaw.

#define V_SOFTSTART_RATE_V_PER_S (600.0f)
#define V_SLEW_UP_RATE_V_PER_S   (1200.0f)
#define V_SLEW_DN_RATE_V_PER_S   (1200.0f)
#define I_SLEW_RATE_A_PER_S      (40.0f)

// Stabilizacja trybu CV/CC.
#define MODE_HYST_PWM_TICKS (120.0f)

// Tłumienie skoków sterowania.
#define PWM_SLEW_UP_TICKS_PER_CYCLE (1200.0f)
#define PWM_SLEW_DN_TICKS_PER_CYCLE (900.0f)
#define PWM_FAST_PULLDOWN_TICKS     (900.0f)
#define PWM_CC_UP_TICKS_PER_CYCLE   (1400.0f)
#define CV_TRACK_ALPHA              (0.35f)

// Miękki filtr pomiarów (EMA).
#define FILT_ALPHA_V (0.22f)
#define FILT_ALPHA_I (0.26f)

// Dodatkowe zachowanie przy przeciążeniu.
#define OC_TRIP_HOLD_TIME_S    (0.0002f)
#define CC_INT_BAND_MIN_A      (0.012f)

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
static float v_set_slewed = 0.0f;
static float i_set_slewed = 0.0f;
static float pwm_cmd = (float)PWM_MIN;
static bool  cc_mode_latched = false;
static bool  prev_output_on = false;
static uint16_t oc_trip_hold_cycles = 0u;
static PID_ControlMode_t control_mode = PID_CONTROL_MODE_OFF;

// Uchwyty
HRTIM_HandleTypeDef *hrtim_ptr = NULL;
ADC_HandleTypeDef   *adc_ptr   = NULL;

// ==========================================
// FUNKCJE POMOCNICZE
// ==========================================

static inline void debug_pin_set(GPIO_PinState state) {
#if (PID_DEBUG_PIN_MODE != 0U)
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, state);
#else
    (void)state;
#endif
}

static inline float clamp(float v, float mn, float mx) {
    if (v < mn) return mn;
    if (v > mx) return mx;
    return v;
}

static inline float ema(float prev, float in, float alpha) {
    return prev + alpha * (in - prev);
}

static inline float slew_towards(float current, float target, float step_up, float step_down) {
    if (target > current) {
        float d = target - current;
        if (d > step_up) d = step_up;
        return current + d;
    }
    float d = current - target;
    if (d > step_down) d = step_down;
    return current - d;
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

PID_ControlMode_t PID_GetControlMode(void) {
    return control_mode;
}

// ==========================================
// RESET (bezpieczny)
// ==========================================
void PID_Reset(void) {
    cv_integral = 0.0f;
    cc_integral = 0.0f;
    v_set_slewed = 0.0f;
    i_set_slewed = 0.0f;
    pwm_cmd = (float)PWM_MIN;
    current_setpoint_v = 0.0f;
    cc_mode_latched = false;
    oc_trip_hold_cycles = 0u;
    control_mode = PID_CONTROL_MODE_OFF;

    if (hrtim_ptr) {
        hrtim_ptr->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = (uint32_t)PWM_MIN;
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
#if (PID_DEBUG_PIN_MODE == 1U)
        debug_pin_set(GPIO_PIN_SET);
#endif
        // 1. Oblicz PID
        PID_HandleInterrupt();

        // 2. Restart ADC
        PID_AdcRestartDMA();

#if (PID_DEBUG_PIN_MODE == 1U)
        debug_pin_set(GPIO_PIN_RESET);
#endif
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
    v_set_slewed = 0.0f;
    i_set_slewed = 0.0f;
    pwm_cmd = (float)PWM_MIN;
    current_setpoint_v = 0.0f;
    cc_mode_latched = false;
    prev_output_on = false;
    oc_trip_hold_cycles = 0u;
    control_mode = PID_CONTROL_MODE_OFF;

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
#if (PID_DEBUG_PIN_MODE == 2U)
    debug_pin_set(GPIO_PIN_SET);
#endif

    // 1) Pomiary + filtracja.
    const float v_in_raw    = (float)adc_raw[0] * COEFF_VOLTAGE;
    const float v_out_raw   = (float)adc_raw[1] * COEFF_VOLTAGE;
    const float i_out_raw   = (float)adc_raw[2] * COEFF_CURRENT;
    const float v_boost_raw = (float)adc_raw[3] * COEFF_VOLTAGE;
    const float i_in_raw    = (float)adc_raw[4] * COEFF_CURRENT;

    const float i_out_fast = (i_out_raw * IOUT_CAL_GAIN) + IOUT_CAL_OFFSET_A;

    v_in    = ema(v_in,    v_in_raw,    FILT_ALPHA_V);
    v_out   = ema(v_out,   v_out_raw + VOUT_CAL_OFFSET_V, FILT_ALPHA_V);
    i_out   = ema(i_out,   i_out_fast,  FILT_ALPHA_I);
    v_boost = ema(v_boost, v_boost_raw, FILT_ALPHA_V);
    i_in    = ema(i_in,    i_in_raw,    FILT_ALPHA_I);

    const bool output_on = hw_output_enabled();

    // 2) OFF / target=0 -> bezpieczny stan.
    if (!output_on || (target_voltage <= 0.0f) || (target_current_lim <= 0.0f)) {
        cv_integral = 0.0f;
        cc_integral = 0.0f;
        v_set_slewed = 0.0f;
        i_set_slewed = 0.0f;
        pwm_cmd = (float)PWM_MIN;
        current_setpoint_v = 0.0f;
        cc_mode_latched = false;
        prev_output_on = output_on;
        oc_trip_hold_cycles = 0u;
        control_mode = PID_CONTROL_MODE_OFF;

        if (hrtim_ptr) {
            hrtim_ptr->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = (uint32_t)PWM_MIN;
            uint32_t cmp3 = ((uint32_t)PWM_MIN) / 2u;
            if (cmp3 < 200u) cmp3 = 200u;
            hrtim_ptr->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP3xR = cmp3;
        }
#if (PID_DEBUG_PIN_MODE == 2U)
        debug_pin_set(GPIO_PIN_RESET);
#endif
        return;
    }

    // 3) Detekcja zbocza ON -> clean soft-start.
    if (!prev_output_on && output_on) {
        cv_integral = 0.0f;
        cc_integral = 0.0f;
        v_set_slewed = 0.0f;
        i_set_slewed = 0.0f;
        pwm_cmd = (float)PWM_MIN;
        cc_mode_latched = false;
        oc_trip_hold_cycles = 0u;
    }
    prev_output_on = output_on;

    // 4) Slew setpointów.
    const float v_target = clamp(target_voltage, 0.0f, 30.0f);
    const float i_target = clamp(target_current_lim, 0.0f, 10.0f);

    const bool soft_start_phase = (v_set_slewed < (v_target - 0.05f));
    const float v_up_rate = soft_start_phase ? V_SOFTSTART_RATE_V_PER_S : V_SLEW_UP_RATE_V_PER_S;

    v_set_slewed = slew_towards(
        v_set_slewed,
        v_target,
        v_up_rate * CTRL_TS_S,
        V_SLEW_DN_RATE_V_PER_S * CTRL_TS_S
    );
    i_set_slewed = slew_towards(
        i_set_slewed,
        i_target,
        I_SLEW_RATE_A_PER_S * CTRL_TS_S,
        I_SLEW_RATE_A_PER_S * CTRL_TS_S
    );
    current_setpoint_v = v_set_slewed;

    // 5) Feed-forward dla buck.
    float pwm_ff = 0.0f;
    if (v_boost > 1.0f) {
        pwm_ff = (v_set_slewed / v_boost) * (float)HRTIM_PERIOD;
    }
    pwm_ff = clamp(pwm_ff, (float)PWM_MIN, (float)PWM_MAX);

    // 6) PI CV + PI CC-limit (klasyczny lab PSU: final = min(CV, CC)).
    const float err_v = v_set_slewed - v_out;
    const float err_i = i_set_slewed - i_out;
    const float u_cv = pwm_ff + (CV_KP * err_v) + cv_integral;
    const float u_cc = pwm_ff + (CC_KP * err_i) + cc_integral;
    const float u_cc_clamped = clamp(u_cc, (float)PWM_MIN, (float)PWM_MAX);

    const float cc_enter_margin = fmaxf(0.0015f, 0.04f * i_set_slewed);
    const float cc_exit_margin  = fmaxf(0.0030f, 0.08f * i_set_slewed);
    const float oc_hard_margin  = fmaxf(0.0030f, 0.10f * i_set_slewed);
    const bool over_i_enter = (i_out_fast > (i_set_slewed + cc_enter_margin));
    const bool under_i_exit = (i_out < (i_set_slewed - cc_exit_margin));
    const bool hard_over_i = (i_out_fast > (i_set_slewed + oc_hard_margin));

    if (over_i_enter) {
        cc_mode_latched = true;
    } else if (cc_mode_latched && under_i_exit) {
        cc_mode_latched = false;
        oc_trip_hold_cycles = 0u;
    }

    float final_unsat = cc_mode_latched ? u_cc : u_cv;
    float final_pwm = clamp(final_unsat, (float)PWM_MIN, (float)PWM_MAX);

    if (over_i_enter) {
        final_pwm = fminf(final_pwm, u_cc_clamped);
    }

    if (hard_over_i) {
        final_pwm = (float)PWM_MIN;
        pwm_cmd = (float)PWM_MIN;
        oc_trip_hold_cycles = (uint16_t)(OC_TRIP_HOLD_TIME_S * CTRL_FS_HZ);
    }
    if (oc_trip_hold_cycles > 0u) {
        final_pwm = (float)PWM_MIN;
        pwm_cmd = (float)PWM_MIN;
        cc_mode_latched = true;
        oc_trip_hold_cycles--;
    }

    // 7) Slew-rate limit PWM.
    float slew_up_ticks = PWM_SLEW_UP_TICKS_PER_CYCLE;
    float slew_dn_ticks = PWM_SLEW_DN_TICKS_PER_CYCLE;
    if (over_i_enter) {
        slew_up_ticks = 0.0f;
        slew_dn_ticks = PWM_FAST_PULLDOWN_TICKS;
    } else if (cc_mode_latched && (err_i > 0.0f)) {
        slew_up_ticks = PWM_CC_UP_TICKS_PER_CYCLE;
    }

    if (!hard_over_i && (oc_trip_hold_cycles == 0u)) {
        pwm_cmd = slew_towards(
            pwm_cmd,
            final_pwm,
            slew_up_ticks,
            slew_dn_ticks
        );
        final_pwm = pwm_cmd;
    }

    // 8) Anti-windup + prowadzenie całek.
    const float aw = PI_AW_GAIN * (final_pwm - final_unsat);
    if (cc_mode_latched) {
        const float cc_int_band = fmaxf(CC_INT_BAND_MIN_A, 0.15f * i_set_slewed);
        if (fabsf(err_i) < cc_int_band) {
            cc_integral += (CC_KI * err_i * CTRL_TS_S) + aw;
        } else {
            cc_integral += aw;
        }
        const float cv_track_target = final_pwm - pwm_ff - (CV_KP * err_v);
        cv_integral += CV_TRACK_ALPHA * (cv_track_target - cv_integral);
    } else {
        cv_integral += (CV_KI * err_v * CTRL_TS_S) + aw;
        const float cc_track_target =
            final_pwm - pwm_ff - (CC_KP * err_i) + (MODE_HYST_PWM_TICKS * 0.6f);
        cc_integral += 0.12f * (cc_track_target - cc_integral);
    }

    cv_integral = clamp(cv_integral, -PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);
    cc_integral = clamp(cc_integral, -PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);
    control_mode = cc_mode_latched ? PID_CONTROL_MODE_CC : PID_CONTROL_MODE_CV;

    // 9) Zapis PWM + trigger ADC.

    final_pwm = clamp(final_pwm, (float)PWM_MIN, (float)PWM_MAX);
    uint32_t u32_pwm = (uint32_t)final_pwm;

    if (hrtim_ptr) {
        hrtim_ptr->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = u32_pwm;
        uint32_t cmp3 = u32_pwm / 2u;
        if (cmp3 < 200u) cmp3 = 200u;
        hrtim_ptr->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP3xR = cmp3;
    }

#if (PID_DEBUG_PIN_MODE == 2U)
    debug_pin_set(GPIO_PIN_RESET);
#endif
}

void PID_SetTargetVoltage(float val) {
    target_voltage = clamp(val, 0.0f, 30.0f);
}

void PID_SetTargetCurrent(float val) {
    target_current_lim = clamp(val, 0.0f, 10.0f);
}

float PID_GetTargetVoltage(void) {
    return target_voltage;
}

float PID_GetTargetCurrent(void) {
    return target_current_lim;
}
