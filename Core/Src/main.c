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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    TIM_HandleTypeDef* pwm_tim;
    uint32_t pwm_channel;

    GPIO_TypeDef* in1_port;
    uint16_t in1_pin;
    GPIO_TypeDef* in2_port;
    uint16_t in2_pin;

    TIM_HandleTypeDef* encoder_tim;
    int32_t encoder_offset;
    int32_t encoder_total;
} Motor;

typedef enum {
    MOTOR_1 = 0,
    MOTOR_2,
    MOTOR_3,
    MOTOR_4,
    MOTOR_COUNT
} Motor_ID;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float max_integral;
} PIDController;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t receivedata[2];
uint8_t message[] = "Hello World";
Motor motors[MOTOR_COUNT];

PIDController pid = {0}; //ǰ����
PIDController pid2 = {0}; //���ҷ���
float target_speed = 50.0f;
uint32_t prev_time;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Debug_Output(const char* movement, int32_t error, float pid_out, float speed1, float speed2);

void Motor_Init(Motor_ID id,
                TIM_HandleTypeDef* pwm_tim, uint32_t pwm_ch,
                GPIO_TypeDef* in1_port, uint16_t in1_pin,
                GPIO_TypeDef* in2_port, uint16_t in2_pin,
                TIM_HandleTypeDef* encoder_tim)
{
    motors[id].pwm_tim = pwm_tim;
    motors[id].pwm_channel = pwm_ch;

    motors[id].in1_port = in1_port;
    motors[id].in1_pin = in1_pin;
    motors[id].in2_port = in2_port;
    motors[id].in2_pin = in2_pin;

    motors[id].encoder_tim = encoder_tim;
    motors[id].encoder_offset = 0;
    motors[id].encoder_total = 0;

    HAL_TIM_PWM_Start(pwm_tim, pwm_ch);

    HAL_TIM_Encoder_Start(encoder_tim, TIM_CHANNEL_ALL);

    motors[id].encoder_offset = (int32_t)__HAL_TIM_GET_COUNTER(encoder_tim);
}

void Motor_SetSpeed(Motor_ID id, int16_t speed)
{
    speed = (speed > 100) ? 100 : (speed < -100) ? -100 : speed;

    if(speed >= 0) {
        HAL_GPIO_WritePin(motors[id].in1_port, motors[id].in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motors[id].in2_port, motors[id].in2_pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(motors[id].in1_port, motors[id].in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motors[id].in2_port, motors[id].in2_pin, GPIO_PIN_SET);
        speed = -speed;
    }

    uint32_t duty = (speed * 999) / 100;
    __HAL_TIM_SET_COMPARE(motors[id].pwm_tim, motors[id].pwm_channel, duty);
}

int32_t Motor_GetEncoder(Motor_ID id)
{
    int32_t current_cnt = (int32_t)__HAL_TIM_GET_COUNTER(motors[id].encoder_tim);
    int32_t diff = current_cnt - motors[id].encoder_offset;
    
    if (diff > 32767) diff -= 65536;
    else if (diff < -32768) diff += 65536;
    
    motors[id].encoder_total += diff;
    
    motors[id].encoder_offset = current_cnt;
    
    return motors[id].encoder_total;
}

float PID_Calculate(PIDController* pid, float error, float dt) {
    float proportional = pid->Kp * error;

    pid->integral += error * dt;
    pid->integral = fmaxf(fminf(pid->integral, pid->max_integral), -pid->max_integral);

    float derivative = pid->Kd * (error - pid->prev_error) / dt;

    float output = proportional + (pid->Ki * pid->integral) + derivative;

    pid->prev_error = error;

    return output;
}

void Motor_Forward(Motor_ID id,Motor_ID id2,int16_t speed){
  uint32_t current_time = HAL_GetTick();
  float dt = (current_time - prev_time) / 1000.0f;
  prev_time = current_time;

  int32_t enc1 = Motor_GetEncoder(id);
  int32_t enc2 = -Motor_GetEncoder(id2);

  int32_t position_error = enc1 - enc2;

  float pid_output = PID_Calculate(&pid, position_error, dt);

  float base_speed = speed;

  float speed1 = base_speed - pid_output;
  float speed2 = -(base_speed + pid_output);
  
  speed1 = fmaxf(fminf(speed1, 100.0f), -100.0f);
	speed2 = fmaxf(fminf(speed2, 100.0f), -100.0f);

  Motor_SetSpeed(id,speed1);
  Motor_SetSpeed(id2,speed2);

  Debug_Output("Forward", position_error, pid_output, speed1, speed2);
}

void Motor_Rightward(Motor_ID id, Motor_ID id2, int16_t speed) {
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - prev_time) / 1000.0f;
    prev_time = current_time;

    int32_t enc1 = Motor_GetEncoder(id);
    int32_t enc2 = -Motor_GetEncoder(id2);

    int32_t position_error = enc1 - enc2;

    float pid_output = PID_Calculate(&pid2, position_error, dt);

    float base_speed = speed;

    float speed1 = base_speed - pid_output;
    float speed2 = -(base_speed + pid_output);
    
    speed1 = fmaxf(fminf(speed1, 100.0f), -100.0f);
    speed2 = fmaxf(fminf(speed2, 100.0f), -100.0f);

    Motor_SetSpeed(id, speed1);
    Motor_SetSpeed(id2, speed2);
    
    Debug_Output("RIGHT", position_error, pid_output, speed1, speed2);
}

void Debug_Output(const char* movement, int32_t error, float pid_out, float speed1, float speed2) {
    static uint32_t last_debug = 0;
    if (HAL_GetTick() - last_debug > 100) {
        char buf[128];
        sprintf(buf, "%s | Err: %4ld | PID: %6.2f | M1: %5.1f%% | M2: %5.1f%%\r\n",
                movement, error, pid_out, speed1, speed2);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
        last_debug = HAL_GetTick();
    }
}

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
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart1, receivedata, 2);
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

  pid.Kp = 2.0f;
  pid.Ki = 0.5f;
  pid.Kd = 0.1f;
  pid.max_integral = 100.0f;
  pid2.Kp = 2.0f;
  pid2.Ki = 0.5f;
  pid2.Kd = 0.1f;
  pid2.max_integral = 100.0f;
  prev_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
