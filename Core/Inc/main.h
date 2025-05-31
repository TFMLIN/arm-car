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
#include "stm32f4xx_hal.h"

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
#define MOTOR1_ENA_Pin GPIO_PIN_0
#define MOTOR1_ENA_GPIO_Port GPIOA
#define MOTOR1_ENB_Pin GPIO_PIN_1
#define MOTOR1_ENB_GPIO_Port GPIOA
#define MOTOR1_IN1_Pin GPIO_PIN_4
#define MOTOR1_IN1_GPIO_Port GPIOA
#define MOTOR1_IN2_Pin GPIO_PIN_5
#define MOTOR1_IN2_GPIO_Port GPIOA
#define LED0_Pin GPIO_PIN_6
#define LED0_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_2
#define LED3_GPIO_Port GPIOB
#define MOTOR1_PWM_Pin GPIO_PIN_9
#define MOTOR1_PWM_GPIO_Port GPIOE
#define MOTOR2_PWM_Pin GPIO_PIN_14
#define MOTOR2_PWM_GPIO_Port GPIOE
#define SERVO_PWM_Pin GPIO_PIN_12
#define SERVO_PWM_GPIO_Port GPIOD
#define MOTOR2_ENA_Pin GPIO_PIN_6
#define MOTOR2_ENA_GPIO_Port GPIOC
#define MOTOR2_ENB_Pin GPIO_PIN_7
#define MOTOR2_ENB_GPIO_Port GPIOC
#define MOTOR2_IN1_Pin GPIO_PIN_8
#define MOTOR2_IN1_GPIO_Port GPIOA
#define MOTOR2_IN2_Pin GPIO_PIN_15
#define MOTOR2_IN2_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
