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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define CSN_Pin GPIO_PIN_3
#define CSN_GPIO_Port GPIOA
#define IRQ_Pin GPIO_PIN_4
#define IRQ_GPIO_Port GPIOA
#define CE_Pin GPIO_PIN_0
#define CE_GPIO_Port GPIOB
#define DIP1_Pin GPIO_PIN_12
#define DIP1_GPIO_Port GPIOB
#define DIP2_Pin GPIO_PIN_13
#define DIP2_GPIO_Port GPIOB
#define DIP3_Pin GPIO_PIN_14
#define DIP3_GPIO_Port GPIOB
#define DIP4_Pin GPIO_PIN_15
#define DIP4_GPIO_Port GPIOB
#define DIP5_Pin GPIO_PIN_8
#define DIP5_GPIO_Port GPIOA
#define DIP6_Pin GPIO_PIN_9
#define DIP6_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
