/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32l4xx_hal.h"

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
#define Valve5_Pin GPIO_PIN_0
#define Valve5_GPIO_Port GPIOA
#define Valve4_Pin GPIO_PIN_6
#define Valve4_GPIO_Port GPIOA
#define Pump3_Pin GPIO_PIN_0
#define Pump3_GPIO_Port GPIOB
#define Pump6_Pin GPIO_PIN_1
#define Pump6_GPIO_Port GPIOB
#define Valve1_Pin GPIO_PIN_8
#define Valve1_GPIO_Port GPIOA
#define Pump1_Pin GPIO_PIN_10
#define Pump1_GPIO_Port GPIOA
#define Valve2_Pin GPIO_PIN_11
#define Valve2_GPIO_Port GPIOA
#define Pump2_Pin GPIO_PIN_12
#define Pump2_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define Valve6_Pin GPIO_PIN_3
#define Valve6_GPIO_Port GPIOB
#define Valve3_Pin GPIO_PIN_5
#define Valve3_GPIO_Port GPIOB
#define Pump5_Pin GPIO_PIN_6
#define Pump5_GPIO_Port GPIOB
#define Pump4_Pin GPIO_PIN_7
#define Pump4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
