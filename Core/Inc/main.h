/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "string.h"
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
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
#define ENCODE_L_B_Pin GPIO_PIN_0
#define ENCODE_L_B_GPIO_Port GPIOC
#define ENCODE_R_B_Pin GPIO_PIN_1
#define ENCODE_R_B_GPIO_Port GPIOC
#define BIN2_Pin GPIO_PIN_1
#define BIN2_GPIO_Port GPIOA
#define ENCODE_L_A_Pin GPIO_PIN_0
#define ENCODE_L_A_GPIO_Port GPIOB
#define ENCODE_L_A_EXTI_IRQn EXTI0_IRQn
#define ENCODE_R_A_Pin GPIO_PIN_1
#define ENCODE_R_A_GPIO_Port GPIOB
#define ENCODE_R_A_EXTI_IRQn EXTI1_IRQn
#define MOTOR_ENABLE_Pin GPIO_PIN_6
#define MOTOR_ENABLE_GPIO_Port GPIOC
#define BIN1_Pin GPIO_PIN_7
#define BIN1_GPIO_Port GPIOC
#define AIN1_Pin GPIO_PIN_8
#define AIN1_GPIO_Port GPIOC
#define AIN2_Pin GPIO_PIN_9
#define AIN2_GPIO_Port GPIOC
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
