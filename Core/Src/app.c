#include "app.h"
#include "gui.h"
#include "main.h"
#include "tcpp.h"

#include "ssd1306.h"

#include "usbd_cdc_if.h"

#include <inttypes.h>
#include <stdio.h>

char usb_msg[128];
uint32_t last_usb_send = 0;
uint16_t msg_len;

static float v_in, v_out, i_out, v_boost, i_in;

float pwm_ff, pwm_pid, pwm_total;
float error;

uint16_t adc_raw[5] = {0};

// Zmienne regulatora
float current_setpoint = 0.0f;  // Aktualnie zadane napięcie (do Soft-Startu)
float pid_integral = 0.0f;      // Pamięć całki
uint8_t protection_tripped = 0; // Flaga błędu

extern ADC_HandleTypeDef hadc1;
extern HRTIM_HandleTypeDef hhrtim1;

float APP_GetCurrentSetpoint() { return current_setpoint; }
float APP_GetVIn() { return v_in; }
float APP_GetVOut() { return v_out; }
float APP_GetIOut() { return i_out; }
float APP_GetVBoost() { return v_boost; }
float APP_GetIIn() { return i_in; }

// Funkcja pomocnicza do bezpiecznego ograniczenia zakresu (Clamp)
float clamp(float value, float min, float max) {
  if (value < min)
    return min;
  if (value > max)
    return max;
  return value;
}

void APP_Init() {
  HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_D);
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);

  // Włączenie drivera
  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, 1);

  // Kalibracja ADC (Dla lepszej precyzji)
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  TCPP_SetPDOSelectionMethod(PDO_SEL_METHOD_MAX_PWR);

  GUI_Init();

  TCPP_Init();
}

void APP_Run() {

  // HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

  // --- 1. ODCZYT ADC (Jak najszybciej) ---
  HAL_ADC_Start(&hadc1);
  for (int i = 0; i < 5; i++) {
    if (HAL_ADC_PollForConversion(&hadc1, 2) == HAL_OK) { // Krótki timeout
      adc_raw[i] = HAL_ADC_GetValue(&hadc1);
    }
  }
  HAL_ADC_Stop(&hadc1);

  // Przeliczenie na jednostki fizyczne
  v_in = (float)adc_raw[0] * COEFF_VOLTAGE;
  v_out = (float)adc_raw[1] * COEFF_VOLTAGE;
  i_out = (float)adc_raw[2] * COEFF_CURRENT;
  v_boost = (float)adc_raw[3] * COEFF_VOLTAGE;
  i_in = (float)adc_raw[4] * COEFF_CURRENT;

  // --- 2. ZABEZPIECZENIA (Protection) ---
  if (v_out > OVP_LIMIT || i_out > OCP_LIMIT) {
    protection_tripped = 1;
    // Wyłączamy driver i PWM
    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, 0);
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);
  }

  // Jeśli zabezpieczenie zadziałało, pomiń regulację
  if (!protection_tripped) {

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
    hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR =
        final_compare;
  }

  // --- 6. TELEMETRIA USB (Co 100ms) ---
  if (HAL_GetTick() - last_usb_send > 100) {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

    if (protection_tripped) {
      msg_len = sprintf(usb_msg, "PROTECTION TRIP! Reset MCU to restart.\r\n");
    } else {
      // Wyświetlanie: Zadane | Vout | Vboost | DutyCycle
      msg_len = sprintf(
          usb_msg,
          "Set:%d.%02dV | Vout:%d.%02dV | Vbst:%d.%02dV | Iout:%d.%02dA | "
          "PWM:%" PRIu32 "\r\n",
          (int)current_setpoint,
          (int)((current_setpoint - (int)current_setpoint) * 100), (int)v_out,
          (int)((v_out - (int)v_out) * 100), (int)v_boost,
          (int)((v_boost - (int)v_boost) * 100), (int)i_out,
          (int)((i_out - (int)i_out) * 100),
          hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR);
    }
    CDC_Transmit_FS((uint8_t *)usb_msg, msg_len);
    last_usb_send = HAL_GetTick();
  }



  GUI_Process();
}