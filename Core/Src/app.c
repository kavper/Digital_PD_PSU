#include "app.h"
#include "app_power.h"
#include "gui.h"
#include "main.h"
#include "stm32g4xx_hal_gpio.h"
#include <stdint.h>

extern TIM_HandleTypeDef htim1;

bool initialized = false;
static const uint32_t app_default_power_w = 100U;

void APP_Init() {
  if (initialized) {
    return;
  }

  PSU_Init();
  GUI_Init();

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  TIM1->CNT = 32768;

  initialized = true;
}

uint32_t APP_GetPower() { return app_default_power_w; }

void APP_Run() {
  if (!initialized) {
    APP_Init();
  }

  if (initialized) {
    GUI_Process();
  }
}
