/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern uint32_t delayTime;
extern void delay_ms(uint32_t del);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LD_FLT_Pin GPIO_PIN_13
#define LD_FLT_GPIO_Port GPIOC
#define LD_BYPSS_Pin GPIO_PIN_14
#define LD_BYPSS_GPIO_Port GPIOC
#define LD_RUN_Pin GPIO_PIN_15
#define LD_RUN_GPIO_Port GPIOC
#define THMO_Pin GPIO_PIN_6
#define THMO_GPIO_Port GPIOA
#define RS_Pin GPIO_PIN_7
#define RS_GPIO_Port GPIOA
#define EN_Pin GPIO_PIN_0
#define EN_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_1
#define D4_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_2
#define D5_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_10
#define D6_GPIO_Port GPIOB
#define D7_Pin GPIO_PIN_11
#define D7_GPIO_Port GPIOB
#define HOB_Pin GPIO_PIN_15
#define HOB_GPIO_Port GPIOB
#define LOB_Pin GPIO_PIN_10
#define LOB_GPIO_Port GPIOA
#define KEY_DWN_Pin GPIO_PIN_11
#define KEY_DWN_GPIO_Port GPIOA
#define KEY_UP_Pin GPIO_PIN_12
#define KEY_UP_GPIO_Port GPIOA
#define KEY_ENT_Pin GPIO_PIN_15
#define KEY_ENT_GPIO_Port GPIOA
#define KEY_MODE_Pin GPIO_PIN_3
#define KEY_MODE_GPIO_Port GPIOB
#define GATE_SD_Pin GPIO_PIN_4
#define GATE_SD_GPIO_Port GPIOB
#define FAN_Pin GPIO_PIN_5
#define FAN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
