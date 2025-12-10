#include "pid.h"
#include "main.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_hrtim.h"

static float v_in, v_out, i_out, v_boost, i_in;

float pwm_ff, pwm_pid, pwm_total;
float error;

uint16_t adc_raw[5] = {0};

// Zmienne regulatora
float current_setpoint = 0.0f;  // Aktualnie zadane napięcie (do Soft-Startu)
float pid_integral = 0.0f;      // Pamięć całki
uint8_t protection_tripped = 0; // Flaga błędu

HRTIM_HandleTypeDef *hrtim;
ADC_HandleTypeDef *adc;

float PID_GetCurrentSetpoint() { return current_setpoint; }
float PID_GetVIn() { return v_in; }
float PID_GetVOut() { return v_out; }
float PID_GetIOut() { return i_out; }
float PID_GetVBoost() { return v_boost; }
float PID_GetIIn() { return i_in; }
float PID_GetPWM() {
  return (float) hrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR / (float) HRTIM_PERIOD;
}

// Funkcja pomocnicza do bezpiecznego ograniczenia zakresu (Clamp)
float clamp(float value, float min, float max) {
  if (value < min)
    return min;
  if (value > max)
    return max;
  return value;
}

void PID_AdcRestartDMA() {
    HAL_ADC_Start_DMA(adc, (uint32_t*)adc_raw, 5);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        // PID_HandleInterrupt();
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
        PID_HandleInterrupt();

        PID_AdcRestartDMA();
    }
}

void PID_Init(ADC_HandleTypeDef *hadc, HRTIM_HandleTypeDef *hhrtim) {
    adc = hadc;
    hrtim = hhrtim;

    HAL_ADC_Start_DMA(adc, (uint32_t*)adc_raw, 5);
}

void PID_HandleInterrupt() {    

  // Przeliczenie na jednostki fizyczne
  v_in = (float)adc_raw[0] * COEFF_VOLTAGE;
  v_out = (float)adc_raw[1] * COEFF_VOLTAGE;
  i_out = (float)adc_raw[2] * COEFF_CURRENT;
  v_boost = (float)adc_raw[3] * COEFF_VOLTAGE;
  i_in = (float)adc_raw[4] * COEFF_CURRENT;

  // --- 3. SOFT START ---
  // Płynnie podnoś napięcie zadane aż do 12V
  if (current_setpoint < V_TARGET_VOLTAGE) {
    current_setpoint += SOFT_START_STEP;
    if (current_setpoint > V_TARGET_VOLTAGE)
      current_setpoint = V_TARGET_VOLTAGE;
  }

  // --- 4. REGULATOR (Feed-Forward + PI) ---

  // A. Feed-Forward (Zgrubne obliczenie)
  // D = V_out / V_in -> PWM = (SetPoint / V_Boost) * Period
  // Zabezpieczenie przed dzieleniem przez zero i zbyt niskim napięciem
  // wejściowym
  if (v_boost > UVLO_LIMIT) {
    pwm_ff = (current_setpoint / v_boost) * HRTIM_PERIOD;
  } else {
    pwm_ff = PWM_MIN;     // Zbyt niskie napięcie zasilania, nie steruj
    current_setpoint = 0; // Reset soft startu
  }

  // B. Regulator PI (Korekta błędu)
  error = current_setpoint - v_out;

  // Całkowanie
  pid_integral += (error * PID_KI);
  // Anti-windup (ograniczenie całki)
  pid_integral = clamp(pid_integral, -PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);

  // Obliczenie korekty PI
  pwm_pid = (error * PID_KP) + pid_integral;

  // C. Sumowanie
  pwm_total = pwm_ff + pwm_pid;

  // D. Ograniczenia wyjścia (Min/Max Duty Cycle)
  uint32_t final_compare = (uint32_t)clamp(pwm_total, PWM_MIN, PWM_MAX);

  // --- 5. AKTUALIZACJA PWM ---
  hrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR =
      final_compare;
}