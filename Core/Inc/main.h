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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef UART_HandleTypeDef	Modbus_UART_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

#define MODBUS_UART_CH	huart2

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RX_EN_Pin GPIO_PIN_1
#define RX_EN_GPIO_Port GPIOA
#define REL_7_Pin GPIO_PIN_4
#define REL_7_GPIO_Port GPIOA
#define REL_6_Pin GPIO_PIN_5
#define REL_6_GPIO_Port GPIOA
#define REL_5_Pin GPIO_PIN_6
#define REL_5_GPIO_Port GPIOA
#define REL_4_Pin GPIO_PIN_7
#define REL_4_GPIO_Port GPIOA
#define REL_3_Pin GPIO_PIN_0
#define REL_3_GPIO_Port GPIOB
#define REL_2_Pin GPIO_PIN_1
#define REL_2_GPIO_Port GPIOB
#define REL_1_Pin GPIO_PIN_2
#define REL_1_GPIO_Port GPIOB
#define REL_0_Pin GPIO_PIN_12
#define REL_0_GPIO_Port GPIOB
#define ACT_LED_Pin GPIO_PIN_13
#define ACT_LED_GPIO_Port GPIOB
#define RX_LED_Pin GPIO_PIN_14
#define RX_LED_GPIO_Port GPIOB
#define TX_LED_Pin GPIO_PIN_15
#define TX_LED_GPIO_Port GPIOB
#define ADD_3_Pin GPIO_PIN_0
#define ADD_3_GPIO_Port GPIOD
#define ADD_2_Pin GPIO_PIN_1
#define ADD_2_GPIO_Port GPIOD
#define ADD_1_Pin GPIO_PIN_2
#define ADD_1_GPIO_Port GPIOD
#define ADD_0_Pin GPIO_PIN_3
#define ADD_0_GPIO_Port GPIOD
#define BAUD_1_Pin GPIO_PIN_3
#define BAUD_1_GPIO_Port GPIOB
#define BAUD_0_Pin GPIO_PIN_4
#define BAUD_0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
