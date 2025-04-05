/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ultrasonic_nonblocking.h"
#include "Servo.h"
#include "motor.h"
#include "mpu6050.h"
#include "pid_controller.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Motor motors[MOTOR_COUNT] = {0};
float target_speed = 50.0f;
uint32_t prev_time = 0;

uint8_t receivedata[2];
uint8_t message[] = "Hello World";
Servo servo1, servo2;

// 定义超声波传感器实例(WARNING)需要修改
UltrasonicSensor ultrasonic_sensor = {
    .trig_port = GPIOA,
    .trig_pin = GPIO_PIN_0,
    .echo_port = GPIOA,
    .echo_pin = GPIO_PIN_1
};

// MPU6050 DMP数据
float pitch, roll, yaw;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    HAL_UART_Transmit(&huart1, message , strlen(message), 100);
    HAL_UART_Receive_IT(&huart1, receivedata, 2);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart1, receivedata, 2);

  // 初始化超声波传感器
  Ultrasonic_Init(&ultrasonic_sensor);

  // 初始化MPU6050 DMP
  int mpu_result = MPU6050_DMP_Init();
  if (mpu_result != 0) {
      char error_msg[50];
      sprintf(error_msg, "MPU6050 DMP初始化失败，错误码: %d\r\n", mpu_result);
      HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, strlen(error_msg), 100);
  } else {
      HAL_UART_Transmit(&huart1, (uint8_t*)"MPU6050 DMP初始化成功\r\n", strlen("MPU6050 DMP初始化成功\r\n"), 100);
  }

  Motor_Init(MOTOR_1,
            &htim5, TIM_CHANNEL_3,
            M1_IN1_GPIO_Port, M1_IN1_Pin,
            M1_IN2_GPIO_Port, M1_IN2_Pin,
            &htim1);

  Motor_Init(MOTOR_2,
            &htim8, TIM_CHANNEL_1,
            M2_IN1_GPIO_Port, M2_IN1_Pin,
            M2_IN2_GPIO_Port, M2_IN2_Pin,
            &htim4);

  Motor_Init(MOTOR_3,
            &htim9, TIM_CHANNEL_1,
            M3_IN1_GPIO_Port, M3_IN1_Pin,
            M3_IN2_GPIO_Port, M3_IN2_Pin,
            &htim3);

  Motor_Init(MOTOR_4,
            &htim10, TIM_CHANNEL_1,
            M4_IN1_GPIO_Port, M4_IN1_Pin,
            M4_IN2_GPIO_Port, M4_IN2_Pin,
            &htim2);

  Servo_Init(&servo1, &htim13, TIM_CHANNEL_1, GPIOF, GPIO_PIN_8);
  Servo_Init(&servo2, &htim14, TIM_CHANNEL_1, GPIOF, GPIO_PIN_9);

  prev_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /*-------------------------------------------------舵机执行部分-------------------------------------------*/
    // Servo_SetAngle(&servo1, 0);    // 0° 舵机测试((WARNING)未完成)

    /*--------------------------------------------------超声波执行部分-------------------------------------*/

    // 更新超声波传感器状态
    // Ultrasonic_Update(&ultrasonic_sensor);
    // // 获取距离数据
    // float distance = Ultrasonic_GetDistance(&ultrasonic_sensor);
    // if (distance > 0) {
    //     char buf[32];
    //     sprintf(buf, "Distance: %.1f cm\r\n", distance);
    //     HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
    // }
    // // 开始下一次测量
    // Ultrasonic_StartMeasurement(&ultrasonic_sensor);

    /*------------------------------------MPU6050 DMP执行部分-------------------------------------*/
    if (MPU6050_DMP_Get_Data(&pitch, &roll, &yaw) == 0) {
        char mpu_buf[64];
        sprintf(mpu_buf, "Pitch: %.2f, Roll: %.2f, Yaw: %.2f\r\n", pitch, roll, yaw);
        HAL_UART_Transmit(&huart1, (uint8_t*)mpu_buf, strlen(mpu_buf), 100);
    }
    target_yaw = yaw;

    /*--------------------------------------电机执行部分---------------------------------------------*/
    // 使用四轮直行控制，速度为50
    Motor_Straight(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, 15);

    HAL_Delay(10);  
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
