#include "app.h"
#include "gui.h"
#include "main.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_adc_ex.h"
#include "tcpp.h"

#include "ssd1306.h"

#include "usbd_cdc_if.h"

#include <inttypes.h>
#include <stdio.h>

#include "3p3z_controller.h"

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
float APP_GetPWM() {
  return hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR;
}

CNTRL_DataFloat CtrlFloat;

// Funkcja pomocnicza do bezpiecznego ograniczenia zakresu (Clamp)
float clamp(float value, float min, float max) {
  if (value < min)
    return min;
  if (value > max)
    return max;
  return value;
}

void APP_HandleADCInterrupt() {
  __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC | ADC_FLAG_OVR);
  uint16_t feedback = LL_ADC_REG_ReadConversionData12( ADC1 );
  CNTRL_3p3zFloat(&CtrlFloat, feedback);

  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_1, CtrlFloat.m_Out);
}

void APP_Init() {

  // Włączenie drivera
  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);

  // Kalibracja ADC (Dla lepszej precyzji)
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  TCPP_SetPDOSelectionMethod(PDO_SEL_METHOD_MAX_PWR);

  GUI_Init();

  TCPP_Init();

  // Inicjalizacja kontrolera
  CNTRL_3p3zFloatInit(&CtrlFloat, REF, A1, A2, A3, B0, B1, B2, B3, K,
                     DUTY_TICKS_MIN, DUTY_TICKS_MAX);

  // Start ADC
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_IT(&hadc1);
  
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);
  HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_D);
  

}

void APP_Run() {
  // Kontroler


  // Blink
  if (HAL_GetTick() - last_usb_send > 100) {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    last_usb_send = HAL_GetTick();
  }

  // GUI_Process();
}