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
#include "OLED.h"
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
uint32_t oled_prev_time = 0;  // 添加OLED刷新时间变量
uint32_t path=0;
uint32_t path_change=0,count_100ms=0;
uint32_t time_start = 0;
float distances[4] = {2000.0f, 2000.0f, 2000.0f, 2000.0f};
float sum[4] = {0, 0, 0, 0};
float mean[4] = {0, 0, 0, 0};
static uint32_t start = 0, now = 0, cz = 1;
static uint32_t start_start = 0,start_now = 0;
bool start_flag = true;
uint32_t time = 0;
bool flag = true;
bool delay_flag = true;

uint8_t receivedata[2];
uint8_t message[] = "Hello World";
uint8_t uart4_rx_buffer;
Servo servo1, servo2 ,servo3 ,servo4 ,servo5;

// 定义超声波传感器实例
// UltrasonicSensor ultrasonic_sensors[5];  // 最多5个超声波传感器
// float ultrasonic_distances[5];  // 存储所有超声波传感器的距离值

// 定义US100传感器实例
US100Sensor us100_sensor1;  // US100传感器实例
US100Sensor us100_sensor2;  // US100传感器实例
US100Sensor us100_sensor3;  // US100传感器实例
US100Sensor us100_sensor4;  // US100传感器实例

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define CLAMP(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))

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

float* meandistances(float* distances)
{

    if (cz)
    {
        cz = 0;
        count_100ms = 1;
        start = HAL_GetTick();
        now = start;
        sum[0] = distances[0];
        sum[1] = distances[1];
        sum[2] = distances[2];
        sum[3] = distances[3];
        return mean;  // 初始返回0值
    }
    else
    {
        now = HAL_GetTick();
        if (now - start <= 100)
        {
            sum[0] += distances[0];
            sum[1] += distances[1];
            sum[2] += distances[2];
            sum[3] += distances[3];
            count_100ms += 1;
            return mean;  // 返回当前均值
        }
        else
        {
            sum[0] += distances[0];
            sum[1] += distances[1];
            sum[2] += distances[2];
            sum[3] += distances[3];
            count_100ms += 1;

            mean[0] = sum[0] / count_100ms;
            mean[1] = sum[1] / count_100ms;
            mean[2] = sum[2] / count_100ms;
            mean[3] = sum[3] / count_100ms;

            cz = 1;
            return mean;
        }
    }
}

#define MAX_SPEED_STEP 5  // 每次最大速度变化量
uint8_t smooth_speed_transition(uint8_t current, uint8_t target) {
    if(target > current) {
        return fmin(current + MAX_SPEED_STEP, target);
    } else if(target < current) {
        return fmax(current - MAX_SPEED_STEP, target);
    }
    return current;
}

void PID_ResetAll(void) {
  PID_Reset(&pid_yaw);
  PID_Reset(&pid_rear);
  PID_Reset(&pid_front);
  PID_Reset(&pid_position);
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
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();

  HAL_UART_Receive_IT(&huart1, receivedata, 2);
  // 初始化UART4接收
  HAL_UART_Receive_IT(&huart4, &uart4_rx_buffer, 1);
  
  HAL_TIM_Base_Start(&htim6);
  Reset_Timer();  // 重置计时器
  
  // 初始化MPU6050 DMP
  int mpu_result;
  int retry_count = 0;
  uint32_t init_start_time = HAL_GetTick();
  
  do {
      mpu_result = MPU6050_DMP_Init();
      if (mpu_result != 0) {
          retry_count++;
          // 只在每10次重试时更新显示，减少OLED操作
          if (retry_count % 10 == 0) {
              OLED_ShowString(1,1,"INITING...");
              OLED_ShowNum(1,11,retry_count,2);
          }
      }
  } while (mpu_result != 0);
  
  OLED_Clear();
  OLED_ShowString(1,1,"SUCCESS");
  // 显示初始化耗时
  uint32_t init_time = HAL_GetTick() - init_start_time;
  OLED_ShowNum(2,1,init_time,4);
  OLED_ShowString(2,5,"ms");

  // 初始化超声波传感器
  // Ultrasonic_Init(&ultrasonic_sensors[1], Trig_1_GPIO_Port, Trig_1_Pin, Echo_1_GPIO_Port, Echo_1_Pin);  // 传感器1
  // Ultrasonic_Init(&ultrasonic_sensors[0], Trig_2_GPIO_Port, Trig_2_Pin, Echo_2_GPIO_Port, Echo_2_Pin);  // 传感器2
  // Ultrasonic_Init(&ultrasonic_sensors[2], Trig_3_GPIO_Port, Trig_3_Pin, Echo_3_GPIO_Port, Echo_3_Pin);  // 传感器3
  // Ultrasonic_Init(&ultrasonic_sensors[3], Trig_4_GPIO_Port, Trig_4_Pin, Echo_4_GPIO_Port, Echo_4_Pin);  // 传感器4
  // Ultrasonic_Init(&ultrasonic_sensors[4], Trig_5_GPIO_Port, Trig_5_Pin, Echo_5_GPIO_Port, Echo_5_Pin);  // 传感器5

  // 初始化US100传感器顺时针1234
  US100_Init(&us100_sensor2, &huart5);
  US100_Init(&us100_sensor1, &huart4);
  US100_Init(&us100_sensor4, &huart3);
  US100_Init(&us100_sensor3, &huart2);
  
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
    OLED_ShowString(3,1,"yaw:");
    OLED_ShowString(3,9,"TAR:");

  
  // 设置目标偏航角为当前偏航角
  target_yaw = yaw;
  
  // 重置PID控制器，避免积分项累积
  PID_Reset(&pid_yaw);
  PID_Reset(&pid_encoder);
  OLED_Clear_Part(1,1,5);
  OLED_ShowString(1, 6, "mm");
  OLED_ShowString(1, 14, "mm");
  OLED_ShowString(2, 6, "mm");
  OLED_ShowString(2, 14, "mm");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  start_start = HAL_GetTick();

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t current_time = HAL_GetTick();
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
    US100_GetAllValidDistances(distances);
    
    while (distances[1]==0)
    {
      US100_GetAllValidDistances(distances);
    }

    if (current_time - oled_prev_time >= 100) {  // 每100ms更新一次显示
        // 显示原始距离和滤波后的距离
        OLED_ShowString(1, 1, "Raw:");
        OLED_ShowString(1, 9, "Filt:");
        OLED_ShowString(2, 1, "Raw:");
        OLED_ShowString(2, 9, "Filt:");
        
        // 显示第一个传感器的原始值和滤波值
        OLED_ShowNum(1, 5, raw_distances[0], 4);
        OLED_ShowNum(1, 13, distances[0], 4);
        
        // 显示第二个传感器的原始值和滤波值
        OLED_ShowNum(2, 5, raw_distances[1], 4);
        OLED_ShowNum(2, 13, distances[1], 4);
        
        oled_prev_time = current_time;
    }

    if(delay_flag) 
    {
      HAL_Delay(500);
      delay_flag=false;
    }
    /*---------------------------------------------------------------电机执行部分---------------------------------------------------------------------------------*/
    // straight_us100(distances[0]);
    // Motor_Rightward(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, 60, &yaw, &target_yaw);
    // Motor_Straight(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, 60, &yaw, &target_yaw);
    // Update_Target_Yaw(&yaw, &target_yaw);
    OLED_ShowChar(3,5,yaw >= 0 ? '+' : '-'); 
    OLED_ShowChar(3,13,target_yaw >= 0 ? '+' : '-'); 
    OLED_ShowNum(3,14,fabsf(target_yaw),3);
    OLED_ShowNum(3,6,fabsf(yaw),3);
    
    OLED_ShowNum(4,1,path,2);  // 显示毫秒
    // OLED_ShowNum(4,4,time,4); OLED_ShowNum(4,10,time_start,4);
    meandistances(distances);


    if (start_flag)
    {
      start_now = HAL_GetTick();
      if (start_now - start_start <= 5000)
      {
          Motor_Straight(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, 60, &yaw, &target_yaw);
          Adjust_Left_Motors_By_Distance(MOTOR_1, MOTOR_3, MOTOR_2, MOTOR_4, distances[0], 30.0f);
      }else{
    	  start_flag = false;
      }
      continue;
    }
    


    switch (path)
    {
    case 0: {
      // 参数定义
      const float TARGET_DISTANCE = 1000.0f;   // 目标保持距离
      const float DECEL_RANGE = 300.0f;      // 减速区间范围（80~180mm）
      const uint8_t MIN_SPEED = 10;          // 最小速度（靠近时）
      const uint8_t MAX_SPEED = 60;          // 最大速度（远端时）
  
      float current_distance = distances[1]; 
  
      // 速度计算逻辑
      uint8_t motor_speed = MAX_SPEED;  // 默认最大速度
  
      if (current_distance <= TARGET_DISTANCE && mean[1] <=TARGET_DISTANCE) {
          // 区域3：到达目标距离（≤80mm）
          motor_speed = MIN_SPEED;
  
          // 执行路径切换逻辑
          if( mean[1] <=100 && current_distance<=100) {
            path += 1;
            PID_ResetAll(); // 重置所有PID控制器
        }
      } 
      else if (current_distance <= (TARGET_DISTANCE + DECEL_RANGE) && mean[1] <= (TARGET_DISTANCE + DECEL_RANGE)) {
          // 区域2：减速区间（70~170mm）
          // 距离越近速度越慢，线性变化：170mm->60, 70mm->10
          float distance_from_target = current_distance - TARGET_DISTANCE;
          float ratio = 1.0f - (distance_from_target / DECEL_RANGE);
          motor_speed = MIN_SPEED + (uint8_t)((MAX_SPEED - MIN_SPEED) * ratio);
          motor_speed = CLAMP(motor_speed, MIN_SPEED, MAX_SPEED);
      }
      else {
          // 区域1：全速区间（>170mm）
          motor_speed = MAX_SPEED;
      }
  
      // 执行带平滑过渡的电机控制
      static uint8_t last_speed = 0;
      motor_speed = smooth_speed_transition(last_speed, motor_speed);
      last_speed = motor_speed;
  
      Motor_Straight(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, motor_speed, &yaw, &target_yaw);
      
      // 使用左侧电机调整
      Adjust_Left_Motors_By_Distance(MOTOR_1, MOTOR_3, MOTOR_2, MOTOR_4, distances[0], 30.0f);
  
      OLED_ShowNum(4, 4, motor_speed, 2);
      break;
  }
    
      case 1: {
        // 参数定义
        const float TARGET_DISTANCE = 80.0f;   // 目标保持距离
        const float DECEL_RANGE = 100.0f;      // 减速区间范围（80~170mm）
        const uint8_t MIN_SPEED = 4;          // 最小速度（靠近时）
        const uint8_t MAX_SPEED = 60;          // 最大速度（远端时）
    
        // 获取当前检测距离（使用第4个传感器）
        float current_distance = distances[3]; 
    
        // 速度计算逻辑
        uint8_t motor_speed = MAX_SPEED;  // 默认最大速度
    
        if (current_distance <= TARGET_DISTANCE) {
            // 区域3：到达目标距离（≤80mm）
            motor_speed = MIN_SPEED;
    
            // 执行路径切换逻辑
            if(current_distance<=32) {
                path += 1;
                PID_ResetAll(); // 重置所有PID控制器
            }
        } 
        else if (current_distance <= (TARGET_DISTANCE + DECEL_RANGE)) {
            // 区域2：减速区间（80~180mm）
            // 距离越近速度越慢，线性变化：180mm->60, 80mm->10
            float distance_from_target = current_distance - TARGET_DISTANCE;
            float ratio = 1.0f - (distance_from_target / DECEL_RANGE);
            motor_speed = MIN_SPEED + (uint8_t)((MAX_SPEED - MIN_SPEED) * ratio);
            motor_speed = CLAMP(motor_speed, MIN_SPEED, MAX_SPEED);
        }
        else {
            // 区域1：全速区间（>170mm）
            motor_speed = MAX_SPEED;
        }
    
        // 执行带平滑过渡的电机控制
        static uint8_t last_speed = 0;
        motor_speed = smooth_speed_transition(last_speed, motor_speed);
        last_speed = motor_speed;
    
        Motor_Rightward(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, motor_speed, &yaw, &target_yaw);
        
        // 使用前后电机调整
        float avg_distance = (distances[1] + distances[2]) / 2.0f;
        Adjust_Motors_By_FrontBack_Distance(MOTOR_1, MOTOR_4, MOTOR_2, MOTOR_3, avg_distance, 30.0f);
    
        OLED_ShowNum(4, 4, motor_speed, 2);
        break;
    }
    
    case 2:
      if (path_change!=2)
      {
        if ((distances[0]>=70&& mean[0]>=70 && path_change==0)||(distances[0]<=70&& mean[0]<=70&& path_change==1))
        {
          Motor_Straight(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, -33, &yaw, &target_yaw);
          // 使用右侧电机调整
          Adjust_Right_Motors_By_Distance(MOTOR_2, MOTOR_4, MOTOR_1, MOTOR_3, distances[3], 30.0f);
        }else if (distances[0]<=100&& mean[0]<=100 && path_change==0)
        {
          if(flag){
            time_start = HAL_GetTick();
            flag = false;
          }
          uint32_t time = HAL_GetTick();
          if(time - time_start >=110){
            path_change+=1;
            flag = true;
          }
        }else if (distances[0]>=70&& mean[0]>=70&& path_change==1)
        {
          if(flag){
            time_start = HAL_GetTick();
            flag = false;
          }
          uint32_t time = HAL_GetTick();
          if(time - time_start >=110){
            path_change+=1;
            flag = true;
          }
        }
      }else{
        path_change = 0;
        path +=1;
        PID_Reset(&pid_yaw);        
        PID_Reset(&pid_rear);
        PID_Reset(&pid_front);
        PID_Reset(&pid_position);
      }
        
      break;

    case 3:{
      // 参数定义
      const float TARGET_DISTANCE = 80.0f;   // 目标保持距离
      const float DECEL_RANGE = 100.0f;      // 减速区间范围（70~170mm）
      const uint8_t MIN_SPEED = -4;          // 最小速度（靠近时）
      const uint8_t MAX_SPEED = -60;          // 最大速度（远端时）
  
      // 获取当前检测距离（使用第4个传感器）
      float current_distance = distances[0]; 
  
      // 速度计算逻辑
      uint8_t motor_speed = MAX_SPEED;  // 默认最大速度
  
      if (current_distance <= TARGET_DISTANCE) {
          // 区域3：到达目标距离（≤70mm）
          motor_speed = MIN_SPEED;
  
          // 执行路径切换逻辑
          if(current_distance<=50) {
            path += 1;
            PID_ResetAll(); // 重置所有PID控制器
          }
      } 
      else if (current_distance <= (TARGET_DISTANCE + DECEL_RANGE)) {
          // 区域2：减速区间（70~170mm）
          // 距离越近速度越慢，线性变化：170mm->60, 70mm->10
          float distance_from_target = current_distance - TARGET_DISTANCE;
          float ratio = 1.0f - (distance_from_target / DECEL_RANGE);
          motor_speed = MIN_SPEED + (uint8_t)((MAX_SPEED - MIN_SPEED) * ratio);
          motor_speed = CLAMP(motor_speed, MIN_SPEED, MAX_SPEED);
      }
      else {
          // 区域1：全速区间（>170mm）
          motor_speed = MAX_SPEED;
      }
  
      // 执行带平滑过渡的电机控制
      static uint8_t last_speed = 0;
      motor_speed = smooth_speed_transition(last_speed, motor_speed);
      last_speed = motor_speed;
  
      Motor_Rightward(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, -motor_speed, &yaw, &target_yaw);
      
      // 使用前后电机调整
      float avg_distance = (distances[1] + distances[2]) / 2.0f;
      Adjust_Motors_By_FrontBack_Distance(MOTOR_1, MOTOR_4, MOTOR_2, MOTOR_3, avg_distance, 30.0f);
  
      OLED_ShowNum(4, 4, motor_speed, 2);
      break;
  }

    case 4:

      if (path_change!=2)
      {
        if ((distances[3]>=70 && mean[3]>=70  && path_change==0)||(distances[3]<=70 && mean[3]<=70 && path_change==1))
        {
          Motor_Straight(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, -30, &yaw, &target_yaw);
          // 使用左侧电机调整
          Adjust_Left_Motors_By_Distance(MOTOR_1, MOTOR_3, MOTOR_2, MOTOR_4, distances[0], 30.0f);
        }else if (distances[3]<=70&& mean[3]<=70&& path_change==0)
        {
          if(flag){
            time_start = HAL_GetTick();
            flag = false;
          }
          uint32_t time = HAL_GetTick();
          if(time - time_start >=100){
            path_change+=1;
            flag = true;
          }
        }else if (distances[3]>=70&& mean[3]>=70&& path_change==1)
        {
          if(flag){
            time_start = HAL_GetTick();
            flag = false;
          }
          uint32_t time = HAL_GetTick();
          if(time - time_start >=100){
            path_change+=1;
            flag = true;
          }
        }
      }else{
        path_change = 0;
        path +=1;
        PID_Reset(&pid_yaw);        
        PID_Reset(&pid_rear);
        PID_Reset(&pid_front);
        PID_Reset(&pid_position);
      }
        
      break;

    case 5:{
      // 参数定义
      const float TARGET_DISTANCE = 80.0f;   // 目标保持距离
      const float DECEL_RANGE = 100.0f;      // 减速区间范围（70~170mm）
      const uint8_t MIN_SPEED = 4;          // 最小速度（靠近时）
      const uint8_t MAX_SPEED = 60;          // 最大速度（远端时）
  
      // 获取当前检测距离（使用第4个传感器）
      float current_distance = distances[3]; 
  
      // 速度计算逻辑
      uint8_t motor_speed = MAX_SPEED;  // 默认最大速度
  
      if (current_distance <= TARGET_DISTANCE) {
          // 区域3：到达目标距离（≤70mm）
          motor_speed = MIN_SPEED;
  
          // 执行路径切换逻辑
          if(current_distance<=50) {
            path += 1;
            PID_ResetAll(); // 重置所有PID控制器
          }
      } 
      else if (current_distance <= (TARGET_DISTANCE + DECEL_RANGE)) {
          // 区域2：减速区间（70~170mm）
          // 距离越近速度越慢，线性变化：170mm->60, 70mm->10
          float distance_from_target = current_distance - TARGET_DISTANCE;
          float ratio = 1.0f - (distance_from_target / DECEL_RANGE);
          motor_speed = MIN_SPEED + (uint8_t)((MAX_SPEED - MIN_SPEED) * ratio);
          motor_speed = CLAMP(motor_speed, MIN_SPEED, MAX_SPEED);
      }
      else {
          // 区域1：全速区间（>170mm）
          motor_speed = MAX_SPEED;
      }
  
      // 执行带平滑过渡的电机控制
      static uint8_t last_speed = 0;
      motor_speed = smooth_speed_transition(last_speed, motor_speed);
      last_speed = motor_speed;
  
      Motor_Rightward(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, motor_speed, &yaw, &target_yaw);
      
      // 使用前后电机调整
      float avg_distance = (distances[1] + distances[2]) / 2.0f;
      Adjust_Motors_By_FrontBack_Distance(MOTOR_1, MOTOR_4, MOTOR_2, MOTOR_3, avg_distance, 30.0f);
  
      OLED_ShowNum(4, 4, motor_speed, 2);
      break;
  }

      case 6:
      if (path_change!=2)
      {
        if ((distances[0]>=70 && mean[0]>=70 && path_change==0)||(distances[0]<=70 && mean[0]<=70 && path_change==1))
        {
          Motor_Straight(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, -30, &yaw, &target_yaw);
          // 使用右侧电机调整
          Adjust_Right_Motors_By_Distance(MOTOR_2, MOTOR_4, MOTOR_1, MOTOR_3, distances[3], 30.0f);
        }else if (distances[0]<=70 && mean[0]<=70 && path_change==0)
        {
          if(flag){
            time_start = HAL_GetTick();
            flag = false;
          }
          uint32_t time = HAL_GetTick();
          if(time - time_start >=100){
            path_change+=1;
            flag = true;
          }
        }else if (distances[0]>=70 && mean[0]>=70 && path_change==1)
        {
          if(flag){
            time_start = HAL_GetTick();
            flag = false;
          }
          uint32_t time = HAL_GetTick();
          if(time - time_start >=100){
            path_change+=1;
            flag = true;
          }
        }
      }else{
        path_change = 0;
        path +=1;
        PID_Reset(&pid_yaw);        
        PID_Reset(&pid_rear);
        PID_Reset(&pid_front);
        PID_Reset(&pid_position);
      }
        
      break;

    case 7:{
        // 参数定义
        const float TARGET_DISTANCE = 80.0f;   // 目标保持距离
        const float DECEL_RANGE = 100.0f;      // 减速区间范围（70~170mm）
        const uint8_t MIN_SPEED = -4;          // 最小速度（靠近时）
        const uint8_t MAX_SPEED = -60;          // 最大速度（远端时）
    
        // 获取当前检测距离（使用第4个传感器）
        float current_distance = distances[0]; 
    
        // 速度计算逻辑
        uint8_t motor_speed = MAX_SPEED;  // 默认最大速度
    
        if (current_distance <= TARGET_DISTANCE) {
            // 区域3：到达目标距离（≤70mm）
            motor_speed = MIN_SPEED;
    
            // 执行路径切换逻辑
            if(current_distance<=50) {
              path += 1;
              PID_ResetAll(); // 重置所有PID控制器
            }
        } 
        else if (current_distance <= (TARGET_DISTANCE + DECEL_RANGE)) {
            // 区域2：减速区间（70~170mm）
            // 距离越近速度越慢，线性变化：170mm->60, 70mm->10
            float distance_from_target = current_distance - TARGET_DISTANCE;
            float ratio = 1.0f - (distance_from_target / DECEL_RANGE);
            motor_speed = MIN_SPEED + (uint8_t)((MAX_SPEED - MIN_SPEED) * ratio);
            motor_speed = CLAMP(motor_speed, MIN_SPEED, MAX_SPEED);
        }
        else {
            // 区域1：全速区间（>170mm）
            motor_speed = MAX_SPEED;
        }
    
        // 执行带平滑过渡的电机控制
        static uint8_t last_speed = 0;
        motor_speed = smooth_speed_transition(last_speed, motor_speed);
        last_speed = motor_speed;
    
        Motor_Rightward(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, motor_speed, &yaw, &target_yaw);
        
        // 使用前后电机调整
        float avg_distance = (distances[1] + distances[2]) / 2.0f;
        Adjust_Motors_By_FrontBack_Distance(MOTOR_1, MOTOR_4, MOTOR_2, MOTOR_3, avg_distance, 30.0f);
    
        OLED_ShowNum(4, 4, motor_speed, 2);
        break;
    }

    case 8:

      if (path_change!=2)
      {
        if ((distances[3]>=70 && mean[3]>=70 && path_change==0)||(distances[3]<=70 && mean[3]<=70 && path_change==1))
        {
          Motor_Straight(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, -30, &yaw, &target_yaw);
          // 使用左侧电机调整
          Adjust_Left_Motors_By_Distance(MOTOR_1, MOTOR_3, MOTOR_2, MOTOR_4, distances[0], 30.0f);
        }else if (distances[3]<=70 && mean[3]<=70 && path_change==0)
        {
          if(flag){
            time_start = HAL_GetTick();
            flag = false;
          }
          uint32_t time = HAL_GetTick();
          if(time - time_start >=100){
            path_change+=1;
            flag = true;
          }
        }else if (distances[3]>=70 && mean[3]>=70 && path_change==1)
        {
          if(flag){
            time_start = HAL_GetTick();
            flag = false;
          }
          uint32_t time = HAL_GetTick();
          if(time - time_start >=100){
            path_change+=1;
            flag = true;
          }
        }
      }else{
        path_change = 0;
        path +=1;
        PID_Reset(&pid_yaw);        
        PID_Reset(&pid_rear);
        PID_Reset(&pid_front);
        PID_Reset(&pid_position);
      }
        
      break;

    case 9:{
      // 参数定义
      const float TARGET_DISTANCE = 80.0f;   // 目标保持距离
      const float DECEL_RANGE = 100.0f;      // 减速区间范围（70~170mm）
      const uint8_t MIN_SPEED = 4;          // 最小速度（靠近时）
      const uint8_t MAX_SPEED = 60;          // 最大速度（远端时）
  
      // 获取当前检测距离（使用第4个传感器）
      float current_distance = distances[3]; 
  
      // 速度计算逻辑
      uint8_t motor_speed = MAX_SPEED;  // 默认最大速度
  
      if (current_distance <= TARGET_DISTANCE) {
          // 区域3：到达目标距离（≤70mm）
          motor_speed = MIN_SPEED;
  
          // 执行路径切换逻辑
          if(current_distance<=50) {
            path += 1;
            PID_ResetAll(); // 重置所有PID控制器
          }
      } 
      else if (current_distance <= (TARGET_DISTANCE + DECEL_RANGE)) {
          // 区域2：减速区间（70~170mm）
          // 距离越近速度越慢，线性变化：170mm->60, 70mm->10
          float distance_from_target = current_distance - TARGET_DISTANCE;
          float ratio = 1.0f - (distance_from_target / DECEL_RANGE);
          motor_speed = MIN_SPEED + (uint8_t)((MAX_SPEED - MIN_SPEED) * ratio);
          motor_speed = CLAMP(motor_speed, MIN_SPEED, MAX_SPEED);
      }
      else {
          // 区域1：全速区间（>170mm）
          motor_speed = MAX_SPEED;
      }
  
      // 执行带平滑过渡的电机控制
      static uint8_t last_speed = 0;
      motor_speed = smooth_speed_transition(last_speed, motor_speed);
      last_speed = motor_speed;
  
      Motor_Rightward(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, motor_speed, &yaw, &target_yaw);
      
      // 使用前后电机调整
      float avg_distance = (distances[1] + distances[2]) / 2.0f;
      Adjust_Motors_By_FrontBack_Distance(MOTOR_1, MOTOR_4, MOTOR_2, MOTOR_3, avg_distance, 30.0f);
  
      OLED_ShowNum(4, 4, motor_speed, 2);
      break;
  }

    case 10:
      if (path_change!=2)
      {
        if ((distances[0]>=70 && mean[0]>=70 && path_change==0)||(distances[0]<=70 && mean[0]<=70 && path_change==1))
        {
          Motor_Straight(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, -30, &yaw, &target_yaw);
          // 使用右侧电机调整
          Adjust_Right_Motors_By_Distance(MOTOR_2, MOTOR_4, MOTOR_1, MOTOR_3, distances[3], 30.0f);
        }else if (distances[0]<=70 && mean[0]<=70 && path_change==0)
        {
          if(flag){
            time_start = HAL_GetTick();
            flag = false;
          }
          uint32_t time = HAL_GetTick();
          if(time - time_start >=100){
            path_change+=1;
            flag = true;
          }
        }else if (distances[0]>=70 && mean[0]>=70 && path_change==1)
        {
          if(flag){
            time_start = HAL_GetTick();
            flag = false;
          }
          uint32_t time = HAL_GetTick();
          if(time - time_start >=100){
            path_change+=1;
            flag = true;
          }
        }
      }else{
        path_change = 0;
        path +=1;
        PID_Reset(&pid_yaw);        
        PID_Reset(&pid_rear);
        PID_Reset(&pid_front);
        PID_Reset(&pid_position);
      }
        
      break;

    case 11:{
      // 参数定义
      const float TARGET_DISTANCE = 80.0f;   // 目标保持距离
      const float DECEL_RANGE = 100.0f;      // 减速区间范围（70~170mm）
      const uint8_t MIN_SPEED = -4;          // 最小速度（靠近时）
      const uint8_t MAX_SPEED = -60;          // 最大速度（远端时）
  
      // 获取当前检测距离（使用第4个传感器）
      float current_distance = distances[0]; 
  
      // 速度计算逻辑
      uint8_t motor_speed = MAX_SPEED;  // 默认最大速度
  
      if (current_distance <= TARGET_DISTANCE) {
          // 区域3：到达目标距离（≤70mm）
          motor_speed = MIN_SPEED;
  
          // 执行路径切换逻辑
          if(current_distance<=50) {
            path += 1;
            PID_ResetAll(); // 重置所有PID控制器
          }
      } 
      else if (current_distance <= (TARGET_DISTANCE + DECEL_RANGE)) {
          // 区域2：减速区间（70~170mm）
          // 距离越近速度越慢，线性变化：170mm->60, 70mm->10
          float distance_from_target = current_distance - TARGET_DISTANCE;
          float ratio = 1.0f - (distance_from_target / DECEL_RANGE);
          motor_speed = MIN_SPEED + (uint8_t)((MAX_SPEED - MIN_SPEED) * ratio);
          motor_speed = CLAMP(motor_speed, MIN_SPEED, MAX_SPEED);
      }
      else {
          // 区域1：全速区间（>170mm）
          motor_speed = MAX_SPEED;
      }
  
      // 执行带平滑过渡的电机控制
      static uint8_t last_speed = 0;
      motor_speed = smooth_speed_transition(last_speed, motor_speed);
      last_speed = motor_speed;
  
      Motor_Rightward(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, motor_speed, &yaw, &target_yaw);
      
      // 使用前后电机调整
      float avg_distance = (distances[1] + distances[2]) / 2.0f;
      Adjust_Motors_By_FrontBack_Distance(MOTOR_1, MOTOR_4, MOTOR_2, MOTOR_3, avg_distance, 30.0f);
  
      OLED_ShowNum(4, 4, motor_speed, 2);
      break;
  }

    case 12:

      if (path_change!=2)
      {
        if ((distances[3]>=70 && mean[3]>=70 && path_change==0)||(distances[3]<=70 && mean[3]<=70 && path_change==1))
        {
          Motor_Straight(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, -30, &yaw, &target_yaw);
          // 使用左侧电机调整
          Adjust_Left_Motors_By_Distance(MOTOR_1, MOTOR_3, MOTOR_2, MOTOR_4, distances[0], 30.0f);
        }else if (distances[3]<=70 && mean[3]<=70 && path_change==0)
        {
          if(flag){
            time_start = HAL_GetTick();
            flag = false;
          }
          uint32_t time = HAL_GetTick();
          if(time - time_start >=100){
            path_change+=1;
            flag = true;
          }
        }else if (distances[3]>=70 && mean[3]>=70 && path_change==1)
        {
          if(flag){
            time_start = HAL_GetTick();
            flag = false;
          }
          uint32_t time = HAL_GetTick();
          if(time - time_start >=100){
            path_change+=1;
            flag = true;
          }
        }
      }else{
        path_change = 0;
        path +=1;
        PID_Reset(&pid_yaw);        
        PID_Reset(&pid_rear);
        PID_Reset(&pid_front);
        PID_Reset(&pid_position);
      }
        
      break;
    }
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
