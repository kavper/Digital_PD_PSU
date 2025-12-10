#include "app.h"
#include "gui.h"
#include "main.h"
#include "tcpp.h"
#include "pid.h"

#include "ssd1306.h"

#include "usbd_cdc_if.h"

#include <inttypes.h>
#include <stdio.h>



extern ADC_HandleTypeDef hadc1;
extern HRTIM_HandleTypeDef hhrtim1;
extern TIM_HandleTypeDef htim2;

bool initialized = false;


void APP_Init() {
  HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_D);
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2); 

  // Kalibracja ADC (Dla lepszej precyzji)
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  // GUI_Init();

  TCPP_SetPDOSelectionMethod(PDO_SEL_METHOD_MAX_PWR);
  TCPP_Init();

  HAL_TIM_Base_Start_IT(&htim2);
  // HAL_ADC_Start_IT(&hadc1);

  PID_Init(&hadc1, &hhrtim1);

  // Włączenie drivera
  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, 1);
  
}

void APP_Run() {
  if (!initialized) {
    APP_Init();
    initialized = true;
  }

  // GUI_Process();
}