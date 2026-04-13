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
#include "stm32f3xx_hal.h"

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
#define ILOAD_Pin GPIO_PIN_0
#define ILOAD_GPIO_Port GPIOA
#define ISCAP_Pin GPIO_PIN_1
#define ISCAP_GPIO_Port GPIOA
#define VOUT_Pin GPIO_PIN_2
#define VOUT_GPIO_Port GPIOA
#define VIN_Pin GPIO_PIN_3
#define VIN_GPIO_Port GPIOA
#define IBAT_Pin GPIO_PIN_4
#define IBAT_GPIO_Port GPIOA
#define VSCAP_Pin GPIO_PIN_5
#define VSCAP_GPIO_Port GPIOA
#define VBUS_Pin GPIO_PIN_6
#define VBUS_GPIO_Port GPIOA
#define RELAY_Pin GPIO_PIN_2
#define RELAY_GPIO_Port GPIOB
#define MOD_Pin GPIO_PIN_10
#define MOD_GPIO_Port GPIOB
#define DRVOFF_Pin GPIO_PIN_12
#define DRVOFF_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
