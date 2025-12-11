/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#include "stm32g4xx_ll_ucpd.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_14
#define LED_GPIO_Port GPIOC
#define V_IN_ADC_Pin GPIO_PIN_0
#define V_IN_ADC_GPIO_Port GPIOA
#define V_OUT_ADC_Pin GPIO_PIN_1
#define V_OUT_ADC_GPIO_Port GPIOA
#define I_OUT_ADC_Pin GPIO_PIN_2
#define I_OUT_ADC_GPIO_Port GPIOA
#define I_IN_ADC_Pin GPIO_PIN_11
#define I_IN_ADC_GPIO_Port GPIOB
#define V_BOOST_ADC_Pin GPIO_PIN_12
#define V_BOOST_ADC_GPIO_Port GPIOB
#define DRV_LI_Pin GPIO_PIN_14
#define DRV_LI_GPIO_Port GPIOB
#define DRV_HI_Pin GPIO_PIN_15
#define DRV_HI_GPIO_Port GPIOB
#define ENC_B_Pin GPIO_PIN_8
#define ENC_B_GPIO_Port GPIOA
#define ENC_A_Pin GPIO_PIN_9
#define ENC_A_GPIO_Port GPIOA
#define ENC_SW_Pin GPIO_PIN_10
#define ENC_SW_GPIO_Port GPIOA
#define ENC_SW_EXTI_IRQn EXTI15_10_IRQn
#define DB_Pin GPIO_PIN_3
#define DB_GPIO_Port GPIOB
#define DRV_EN_Pin GPIO_PIN_9
#define DRV_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
