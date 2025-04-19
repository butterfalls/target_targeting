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
#include "us100_uart.h"
#include <stdio.h>
#include <string.h>
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
uint8_t uart4_rx_buffer;
Servo servo1, servo2 ,servo3 ,servo4 ,servo5;

static float last_valid_distance1 = 0;
static float last_valid_distance2 = 0;
static float last_valid_distance3 = 0;
static float last_valid_distance4 = 0;


// 定义超声波传感器实例
// UltrasonicSensor ultrasonic_sensors[5];  // 最多5个超声波传感器
// float ultrasonic_distances[5];  // 存储所有超声波传感器的距离值

// 定义US100传感器实例
US100Sensor us100_sensor1;  // US100传感器实例
US100Sensor us100_sensor2;  // US100传感器实例
US100Sensor us100_sensor3;  // US100传感器实例
US100Sensor us100_sensor4;  // US100传感器实例

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
    // 检查是否是UART5（US100传感器使用的串口）
    if (huart == &huart5||huart == &huart2||huart == &huart3||huart == &huart4) {
        // 调用US100库的回调函数
        US100_UART_RxCpltCallback(huart);
    } 
    if (huart == &huart1) {
        // 处理其他串口的回调
        HAL_UART_Transmit(&huart1, message, strlen(message), 100);
        HAL_UART_Receive_IT(&huart1, receivedata, 2);
    }
    // if (huart == &huart4) {
    //     // 输出调试信息
    //     char debug_msg[50];
    //     sprintf(debug_msg, "UART4 received: 0x%02X\r\n", uart4_rx_buffer);
    //     HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), 100);
        
    //     // 继续接收下一个字节
    //     HAL_UART_Receive_IT(&huart4, &uart4_rx_buffer, 1);
    // }
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart1, receivedata, 2);
  // 初始化UART4接收
  HAL_UART_Receive_IT(&huart4, &uart4_rx_buffer, 1);
  
// 初始化MPU6050 DMP
  int mpu_result;
  int retry_count = 0;
  // do {
      mpu_result = MPU6050_DMP_Init();
  //     if (mpu_result != 0) {
  //         char error_msg[100];
  //         sprintf(error_msg, "MPU6050 DMP 初始化失败，错误码: %d，重试次数: %d\r\n", mpu_result, retry_count);
  //         HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, strlen(error_msg), 100);
          
  //         // 等待一段时间后重试
  //         HAL_Delay(500); // 等待500ms
  //         retry_count++;
  //     } else {
  //         char success_msg[100];
  //         sprintf(success_msg, "MPU6050 初始化成功，共尝试 %d 次\r\n", retry_count + 1);
  //         HAL_UART_Transmit(&huart1, (uint8_t*)success_msg, strlen(success_msg), 100);
  //     }
  // } while (mpu_result != 0);

  // 初始化超声波传感器
  // Ultrasonic_Init(&ultrasonic_sensors[1], Trig_1_GPIO_Port, Trig_1_Pin, Echo_1_GPIO_Port, Echo_1_Pin);  // 传感器1
  // Ultrasonic_Init(&ultrasonic_sensors[0], Trig_2_GPIO_Port, Trig_2_Pin, Echo_2_GPIO_Port, Echo_2_Pin);  // 传感器2
  // Ultrasonic_Init(&ultrasonic_sensors[2], Trig_3_GPIO_Port, Trig_3_Pin, Echo_3_GPIO_Port, Echo_3_Pin);  // 传感器3
  // Ultrasonic_Init(&ultrasonic_sensors[3], Trig_4_GPIO_Port, Trig_4_Pin, Echo_4_GPIO_Port, Echo_4_Pin);  // 传感器4
  // Ultrasonic_Init(&ultrasonic_sensors[4], Trig_5_GPIO_Port, Trig_5_Pin, Echo_5_GPIO_Port, Echo_5_Pin);  // 传感器5

  // 初始化US100传感器
  US100_Init(&us100_sensor1, &huart4);
  US100_Init(&us100_sensor2, &huart5);
  US100_Init(&us100_sensor3, &huart2);
  US100_Init(&us100_sensor4, &huart3);
  
  // 等待一段时间，确保传感器稳定
  HAL_Delay(50);
  
  // 开始第一次测量
  US100_StartMeasurement(&us100_sensor1);
  US100_StartMeasurement(&us100_sensor2);
  US100_StartMeasurement(&us100_sensor3);
  US100_StartMeasurement(&us100_sensor4);

  Motor_Init(MOTOR_1,
            &htim5, TIM_CHANNEL_1,
            M1_IN1_GPIO_Port, M1_IN1_Pin,
            M1_IN2_GPIO_Port, M1_IN2_Pin,
            &htim1);

  Motor_Init(MOTOR_2,
            &htim5, TIM_CHANNEL_2,
            M2_IN1_GPIO_Port, M2_IN1_Pin,
            M2_IN2_GPIO_Port, M2_IN2_Pin,
            &htim4);

  Motor_Init(MOTOR_3,
            &htim5, TIM_CHANNEL_3,
            M3_IN1_GPIO_Port, M3_IN1_Pin,
            M3_IN2_GPIO_Port, M3_IN2_Pin,
            &htim3);

  Motor_Init(MOTOR_4,
            &htim5, TIM_CHANNEL_4,
            M4_IN1_GPIO_Port, M4_IN1_Pin,
            M4_IN2_GPIO_Port, M4_IN2_Pin,
            &htim2);

  Servo_Init(&servo1, &htim8, TIM_CHANNEL_1, Servo_1_GPIO_Port, Servo_1_Pin);
  Servo_Init(&servo2, &htim8, TIM_CHANNEL_2, Servo_2_GPIO_Port, Servo_2_Pin);
  Servo_Init(&servo3, &htim9, TIM_CHANNEL_1, Servo_3_GPIO_Port, Servo_3_Pin);
  Servo_Init(&servo4, &htim9, TIM_CHANNEL_2, Servo_4_GPIO_Port, Servo_4_Pin);
  Servo_Init(&servo5, &htim10, TIM_CHANNEL_1, Servo_5_GPIO_Port, Servo_5_Pin);

  prev_time = HAL_GetTick();

  /*------------------------------------MPU6050 DMP执行部分-------------------------------------*/
  if (MPU6050_DMP_Get_Data(&pitch, &roll, &yaw) == 0) {
      char mpu_buf[64];
      sprintf(mpu_buf, "Pitch: %.2f, Roll: %.2f, Yaw: %.2f\r\n", pitch, roll, yaw);
      HAL_UART_Transmit(&huart1, (uint8_t*)mpu_buf, strlen(mpu_buf), 100);
  }
  
  // 设置目标偏航角为当前偏航角
  target_yaw = yaw;
  
  // 重置PID控制器，避免积分项累积
  PID_Reset(&pid_yaw);
  PID_Reset(&pid_encoder);
  
  char debug_buf[64];
  sprintf(debug_buf, "target_yaw: %.2f\r\n", target_yaw);
  HAL_UART_Transmit(&huart1, (uint8_t*)debug_buf, strlen(debug_buf), 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /*------------------------------------------------------------------------舵机执行部分--------------------------------------------------------------------*/
    // Servo_SetAngle(&servo3, 0);
    // HAL_Delay(200);
    // Servo_SetAngle(&servo3, 60);
    // HAL_Delay(200);
    

    /*-----------------------------------------------------------------超声波执行部分（暂不使用）-------------------------------------------------------------------------*/

    // 更新超声波传感器状态
    // Ultrasonic_Update(&ultrasonic_sensors[0]);
    
    // 获取超声波传感器的距离值
    // float distance = Ultrasonic_GetDistance(&ultrasonic_sensors[0]);
    
    // 输出超声波传感器的距离值
    // if (distance > 0) {
    //     char buf[32];
    //     sprintf(buf, "US1: %.1f cm\r\n", distance);
    //     HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
    // }
    
    // 开始下一次测量
    // Ultrasonic_StartMeasurement(&ultrasonic_sensors[0]);
    
    // 添加延时，确保超声波传感器有足够的时间完成测量
    // HAL_Delay(50);  // 增加延时到50ms，给传感器更多恢复时间

    /*----------------------------------------------------------------------------US100传感器执行部分-------------------------------------------------------------*/
    US100_Update(&us100_sensor1);
    US100_Update(&us100_sensor2);
    US100_Update(&us100_sensor3);
    US100_Update(&us100_sensor4);
        
    float us100_distance1 = US100_GetDistance(&us100_sensor1);
    float us100_distance2 = US100_GetDistance(&us100_sensor2);
    float us100_distance3 = US100_GetDistance(&us100_sensor3);
    float us100_distance4 = US100_GetDistance(&us100_sensor4);
    
    if (us100_distance1 > 0) last_valid_distance1 = us100_distance1;
    if (us100_distance2 > 0) last_valid_distance2 = us100_distance2;
    if (us100_distance3 > 0) last_valid_distance3 = us100_distance3;
    if (us100_distance4 > 0) last_valid_distance4 = us100_distance4;
    
    if (last_valid_distance1 > 0 && last_valid_distance2 > 0 && last_valid_distance3 > 0 && last_valid_distance4 > 0) {
        char buf[128];
        sprintf(buf, "US100: %.0f, %.0f, %.0f, %.0f mm\r\n", 
                last_valid_distance1, last_valid_distance2, last_valid_distance3, last_valid_distance4);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
        
        // 开始下一次测量
        US100_StartMeasurement(&us100_sensor1);
        US100_StartMeasurement(&us100_sensor2);
        US100_StartMeasurement(&us100_sensor3);
        US100_StartMeasurement(&us100_sensor4);
    } 
    else if ((us100_distance1 == 0||us100_distance1 == -1) && (us100_distance2 == 0||us100_distance2 == -1) && 
             (us100_distance3 == 0||us100_distance3 == -1) && (us100_distance4 == 0||us100_distance4 == -1)){
        static uint32_t last_measurement_time = 0;
        static uint8_t timeout_count = 0;
        
        uint32_t current_time = HAL_GetTick();
        
        if (current_time - last_measurement_time > 300) {
            timeout_count++;
            
            // 输出调试信息
            char debug_buf[64];
            sprintf(debug_buf, "US100: No valid data for %d times\r\n", timeout_count);
            HAL_UART_Transmit(&huart1, (uint8_t*)debug_buf, strlen(debug_buf), 100);
            
            if (timeout_count >= 20) {
                US100_StartMeasurement(&us100_sensor1);
                US100_StartMeasurement(&us100_sensor2);
                US100_StartMeasurement(&us100_sensor3);
                US100_StartMeasurement(&us100_sensor4);
                timeout_count = 0;
            }
            
            last_measurement_time = current_time;
        }
    }

    /*---------------------------------------------------------------电机执行部分---------------------------------------------------------------------------------*/
    straight_us100(last_valid_distance1);

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
