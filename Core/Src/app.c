#include "app.h"
#include "gui.h"
#include "main.h"
#include "pid.h"
#include "stm32g4xx_hal_gpio.h"
#include "tcpp.h"
#include "ssd1306.h"
#include "usbd_cdc_if.h"
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>

extern ADC_HandleTypeDef hadc1;
extern HRTIM_HandleTypeDef hhrtim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim1;

bool initialized = false;

void APP_Init() {
  HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_D);
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);

  // Kalibracja ADC (Dla lepszej precyzji)
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  GUI_Init();

  TCPP_SetPDOSelectionMethod(PDO_SEL_METHOD_MAX_PWR);
  TCPP_Init();

  HAL_TIM_Base_Start_IT(&htim2);
  // HAL_ADC_Start_IT(&hadc1);

  PID_Init(&hadc1, &hhrtim1);

  // Włączenie drivera
  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, 1);

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

  TIM1->CNT = 32768;
}

extern USBPD_HandleTypeDef DPM_Ports[];

USBPD_SRCFixedSupplyPDO_TypeDef pdosy[32];

volatile uint32_t curr;
volatile uint32_t volt;

uint32_t voltageVolts;
uint32_t currentMiliAmps;
uint32_t powerWatts;

uint32_t currentPower = 0;

uint32_t APP_GetPower() { return currentPower; }

void APP_Run() {

  TCPP_SetPDOSelectionMethod(PDO_SEL_METHOD_MAX_PWR);
  TCPP_Init();

  uint8_t pdosnum = DPM_Ports[USBPD_PORT_0].DPM_NumberOfRcvSRCPDO;

  for (uint32_t i = 0; i < pdosnum; i++) {
    uint32_t raw_pdo = DPM_Ports[USBPD_PORT_0].DPM_ListOfRcvSRCPDO[i];
    USBPD_PDO_TypeDef pdo;
    pdo.d32 = raw_pdo;

    switch (pdo.GenericPDO.PowerObject) {
    case USBPD_CORE_PDO_TYPE_FIXED: {
      USBPD_SRCFixedSupplyPDO_TypeDef fixed = pdo.SRCFixedPDO;
      pdosy[i] = fixed;

      voltageVolts = (float)fixed.VoltageIn50mVunits * 0.05f; // przelicz na V
      currentMiliAmps =
          (float)fixed.MaxCurrentIn10mAunits * 10.0f; // przelicz na A

      powerWatts = voltageVolts * (currentMiliAmps / 1000.0f);

      if (voltageVolts * 1000 == DPM_Ports[0].DPM_RequestedVoltage)
        currentPower = powerWatts;

      break;
    }

    default:
      break;
    }
  }

  // if (pdosnum >= 1 && !initialized)
  //   return;

  static uint32_t gui_start_tick = 0;

  if (!initialized) {
    APP_Init();
    initialized = true;
    gui_start_tick = HAL_GetTick();
  }

  if (initialized && (HAL_GetTick() - gui_start_tick) >= 1000) {
    GUI_Process();
  }
}