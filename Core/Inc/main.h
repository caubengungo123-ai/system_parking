/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define mq2_Pin GPIO_PIN_0
#define mq2_GPIO_Port GPIOA
#define cs_Pin GPIO_PIN_4
#define cs_GPIO_Port GPIOA
#define rst_Pin GPIO_PIN_0
#define rst_GPIO_Port GPIOB
#define ky_026_Pin GPIO_PIN_1
#define ky_026_GPIO_Port GPIOB
#define nut_bam_2_Pin GPIO_PIN_8
#define nut_bam_2_GPIO_Port GPIOA
#define nut_bam_1_Pin GPIO_PIN_9
#define nut_bam_1_GPIO_Port GPIOA
#define servo2_Pin GPIO_PIN_15
#define servo2_GPIO_Port GPIOA
#define servo1_Pin GPIO_PIN_3
#define servo1_GPIO_Port GPIOB
#define buzzer_Pin GPIO_PIN_8
#define buzzer_GPIO_Port GPIOB
#define led_Pin GPIO_PIN_9
#define led_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
