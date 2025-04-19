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
#define Servo_3_Pin GPIO_PIN_5
#define Servo_3_GPIO_Port GPIOE
#define Servo_4_Pin GPIO_PIN_6
#define Servo_4_GPIO_Port GPIOE
#define Servo_5_Pin GPIO_PIN_6
#define Servo_5_GPIO_Port GPIOF
#define M4_IN1_Pin GPIO_PIN_1
#define M4_IN1_GPIO_Port GPIOC
#define M4_IN2_Pin GPIO_PIN_3
#define M4_IN2_GPIO_Port GPIOC
#define M1_PWM_Pin GPIO_PIN_0
#define M1_PWM_GPIO_Port GPIOA
#define M2_PWM_Pin GPIO_PIN_1
#define M2_PWM_GPIO_Port GPIOA
#define M3_PWM_Pin GPIO_PIN_2
#define M3_PWM_GPIO_Port GPIOA
#define M4_PWM_Pin GPIO_PIN_3
#define M4_PWM_GPIO_Port GPIOA
#define M4_E1_Pin GPIO_PIN_5
#define M4_E1_GPIO_Port GPIOA
#define M3_E1_Pin GPIO_PIN_6
#define M3_E1_GPIO_Port GPIOA
#define M3_E2_Pin GPIO_PIN_7
#define M3_E2_GPIO_Port GPIOA
#define M3_IN1_Pin GPIO_PIN_5
#define M3_IN1_GPIO_Port GPIOC
#define M3_IN2_Pin GPIO_PIN_1
#define M3_IN2_GPIO_Port GPIOB
#define M1_IN1_Pin GPIO_PIN_0
#define M1_IN1_GPIO_Port GPIOG
#define M1_IN2_Pin GPIO_PIN_7
#define M1_IN2_GPIO_Port GPIOE
#define M1_E1_Pin GPIO_PIN_9
#define M1_E1_GPIO_Port GPIOE
#define M1_E2_Pin GPIO_PIN_11
#define M1_E2_GPIO_Port GPIOE
#define Trig_4_Pin GPIO_PIN_14
#define Trig_4_GPIO_Port GPIOE
#define Trig_3_Pin GPIO_PIN_15
#define Trig_3_GPIO_Port GPIOE
#define Echo_4_Pin GPIO_PIN_10
#define Echo_4_GPIO_Port GPIOB
#define Echo_4_EXTI_IRQn EXTI15_10_IRQn
#define Echo_3_Pin GPIO_PIN_11
#define Echo_3_GPIO_Port GPIOB
#define Echo_3_EXTI_IRQn EXTI15_10_IRQn
#define M2_E1_Pin GPIO_PIN_12
#define M2_E1_GPIO_Port GPIOD
#define M2_E2_Pin GPIO_PIN_13
#define M2_E2_GPIO_Port GPIOD
#define M2_IN2_Pin GPIO_PIN_15
#define M2_IN2_GPIO_Port GPIOD
#define M2_IN1_Pin GPIO_PIN_3
#define M2_IN1_GPIO_Port GPIOG
#define Servo_1_Pin GPIO_PIN_6
#define Servo_1_GPIO_Port GPIOC
#define Servo_2_Pin GPIO_PIN_7
#define Servo_2_GPIO_Port GPIOC
#define TX_1_Pin GPIO_PIN_10
#define TX_1_GPIO_Port GPIOC
#define RX_1_Pin GPIO_PIN_11
#define RX_1_GPIO_Port GPIOC
#define Echo_2_Pin GPIO_PIN_13
#define Echo_2_GPIO_Port GPIOG
#define Echo_2_EXTI_IRQn EXTI15_10_IRQn
#define Trig_2_Pin GPIO_PIN_15
#define Trig_2_GPIO_Port GPIOG
#define M4_E2_Pin GPIO_PIN_3
#define M4_E2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
