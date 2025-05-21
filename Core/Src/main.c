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
uint8_t aRxBuffer[1];
uint8_t ledState = 0;
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

// 添加各个case的进入时间记录
static uint32_t time_enterpath_case0 = 0;
static uint32_t time_enterpath_case1 = 0;
static uint32_t time_enterpath_case2 = 0;
static uint32_t time_enterpath_case3 = 0;
static uint32_t time_enterpath_case4 = 0;
static uint32_t time_enterpath_case5 = 0;
static uint32_t time_enterpath_case6 = 0;
static uint32_t time_enterpath_case7 = 0;
static uint32_t time_enterpath_case8 = 0;
static uint32_t time_enterpath_case9 = 0;
static uint32_t time_enterpath_case10 = 0;
static uint32_t time_enterpath_case11 = 0;

uint8_t OpenMVdata = 0;  // 修改为变量声明
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
    if (huart == &huart1) {
    ledState = !ledState;
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, ledState ? GPIO_PIN_SET : GPIO_PIN_RESET);
    // 继续接收下一个字节
    HAL_UART_Receive_IT(&huart1, aRxBuffer, 1);
    }
    if (huart == &huart5||huart == &huart2||huart == &huart3||huart == &huart4) {
        // 调用US100库的回调函数
        US100_UART_RxCpltCallback(huart);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);  // 收到'a'后关闭LED
    } 
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


void Rotate_90_Degrees(Motor_ID id1, Motor_ID id2, Motor_ID id3, Motor_ID id4, bool clockwise) {
  float ROTATION_SPEED;  // 旋转速度
  static const float ROTATION_SPEED_max = 30.0f;
  static const float ANGLE_TOLERANCE = 6.9f;  // 角度容差
  // MPU6050_DMP_Get_Data(&pitchstart , &rollopen , &yaw);
  float const start_yaw = target_yaw;  // 记录起始角度
  float target_angle = start_yaw + (clockwise ? -90.0f : 90.0f);  // 计算目标角度
  unsigned int num = 0;
  
  // 标准化目标角度到-180到180度范围
  if (target_angle > 180.0f) {
      target_angle -= 360.0f;
  } else if (target_angle < -180.0f) {
      target_angle += 360.0f;
  }
  
  // 设置目标偏航角
  target_yaw = target_angle;
  
  // // 重置PID控制器
  // PID_Reset(&pid_yaw);
  // PID_Reset(&pid_front);
  // PID_Reset(&pid_rear);
  // PID_Reset(&pid_position);

  
  // 开始旋转
  while (num <= 20) {
      // 获取当前偏航角
      float pitch, roll, current_yaw;
      if (MPU6050_DMP_Get_Data(&pitch, &roll, &current_yaw) != 0) {
          continue;  // 如果获取数据失败，继续尝试
      }

      OLED_ShowChar(3,5,current_yaw >= 0 ? '+' : '-'); 
      OLED_ShowChar(3,13,target_yaw >= 0 ? '+' : '-'); 
      OLED_ShowNum(3,14,fabsf(target_yaw),3);
      OLED_ShowNum(3,6,fabsf(current_yaw),3);
      OLED_ShowChar(4,5,roll >= 0 ? '+' : '-');
      OLED_ShowNum(4, 6, fabsf(roll) , 3);
      
      // 计算角度误差
      float angle_error = target_angle - current_yaw;
      float voltage=angle_error*1.12-roll*0.3;

      // 根据电压的正负分别限制速度，且要求绝对值大于12
      if (fabsf(voltage) > 10.0f) {
          if (voltage >= 0) {
              ROTATION_SPEED = fmin(voltage, ROTATION_SPEED_max);
          } else {
              ROTATION_SPEED = fmax(voltage, -ROTATION_SPEED_max);
          }
      } else {
          // 如果电压绝对值小于等于12，保持原符号但设为12
          ROTATION_SPEED = (voltage >= 0) ? 10.0f : -10.0f;
      }
      // 如果达到目标角度（考虑容差），停止旋转
      
      // 根据旋转方向设置电机速度
      Motor_SetSpeed(id1, ROTATION_SPEED);  // 左前
      Motor_SetSpeed(id2, ROTATION_SPEED);  // 右后
      Motor_SetSpeed(id3, -ROTATION_SPEED);   // 左后
      Motor_SetSpeed(id4, -ROTATION_SPEED);   // 右前

      if (fabsf(angle_error) <= ANGLE_TOLERANCE){
          num++;
      }
      HAL_Delay(10);
  }

  // 停止所有电机
  Motor_SetSpeed(id1, 0);  // 左前
  Motor_SetSpeed(id2, 0);  // 右后
  Motor_SetSpeed(id3, 0);  // 左后
  Motor_SetSpeed(id4, 0);  // 右前
}

void Servo_open_red_left()
{
    // 使用正确的舵机变量名和角度
    Servo_SetAngle(&servo1, 115);
    Servo_SetAngle(&servo2, 122);
    Servo_SetAngle(&servo3, 38);
    Servo_SetAngle(&servo4, 48);
    Servo_SetAngle(&servo5, 15);
}

void Servo_open_red_right()
{
    // 使用正确的舵机变量名和角度
    Servo_SetAngle(&servo1, 35);
    Servo_SetAngle(&servo2, 35);
    Servo_SetAngle(&servo3, 123);
    Servo_SetAngle(&servo4, 138);
    Servo_SetAngle(&servo5, 15);
}

void Servo_open_green_left()
{
    // 使用正确的舵机变量名和角度
    Servo_SetAngle(&servo1, 35);
    Servo_SetAngle(&servo2, 35);
    Servo_SetAngle(&servo3, 123);
    Servo_SetAngle(&servo4, 48);
    Servo_SetAngle(&servo5, 15);
}

void Servo_open_green_right()
{
    // 使用正确的舵机变量名和角度
    Servo_SetAngle(&servo1, 35);
    Servo_SetAngle(&servo2, 35);
    Servo_SetAngle(&servo3, 38);
    Servo_SetAngle(&servo4, 138);
    Servo_SetAngle(&servo5, 15);
}

void Servo_close()
{
    // 使用正确的舵机变量名和角度
    Servo_SetAngle(&servo5, 105);
    Servo_SetAngle(&servo1, 35);
    Servo_SetAngle(&servo2, 35);
    Servo_SetAngle(&servo3, 38);
    Servo_SetAngle(&servo4, 48);
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

  // 修改为UART6的接收中断初始化
  HAL_UART_Receive_IT(&huart6, &OpenMVdata, 1); 
  HAL_UART_Receive_IT(&huart1, &OpenMVdata, 1); 
  
  HAL_TIM_Base_Start(&htim6);
  Reset_Timer();  // 重置计时器
  
  // while(1){
  // HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
  // HAL_Delay(1000);
  // HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
  // HAL_Delay(1000);
  // }

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
  // OLED_ShowString(1, 6, "mm");
  // OLED_ShowString(1, 14, "mm");
  // OLED_ShowString(2, 6, "mm");
  // OLED_ShowString(2, 14, "mm");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  start_start = HAL_GetTick();
  // OLED_Clear();
  // while(1)
  // {
  //   // 发送0x55命令到sensor2
  //   uint8_t cmd = 0x55;
  //   uint8_t rx_data[2] = {0};  // 用于存储接收到的数据
    
  //   HAL_UART_Transmit(&huart4, &cmd, 1, 100);
    
  //   // 接收数据
  //   if(HAL_UART_Receive(&huart4, rx_data, 2, 1000) == HAL_OK) {
  //     // 计算距离值：低字节在前，高字节在后
  //     uint16_t distance = (rx_data[1] << 8) | rx_data[0];
      
  //     // 显示原始数据用于调试
  //     OLED_ShowNum(1,1,rx_data[0],3);  // 显示低字节
  //     OLED_ShowNum(1,5,rx_data[1],3);  // 显示高字节
  //     OLED_ShowNum(2,1,rx_data[1] << 8,3);    // 显示左移8位后的值
  //     OLED_ShowNum(2,5,distance,5);    // 显示最终距离值
  //   }
    
  //   HAL_Delay(1000);
  //   OLED_Clear();
  // }

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t current_time = HAL_GetTick();
    /*------------------------------------------------------------------------舵机执行部分--------------------------------------------------------------------*/
    static uint8_t last_data = 0;
    static uint32_t last_time = 0;
    OLED_ShowNum(4,13,aRxBuffer[0],2);

    bool can_change_state = (current_time - last_time >= 800);

    if(last_data != aRxBuffer[0] && can_change_state) {
        if(aRxBuffer[0] == 48) {
            Servo_close();
        } else if(aRxBuffer[0] == 49) {
            if(path == 3 || path == 7 || path == 11) {
                Servo_open_red_left();
            } else if(path == 5 || path == 9) {
                Servo_open_red_right();
            }
        } else if(aRxBuffer[0] == 50) {
            if(path == 3 || path == 7 || path == 11) {
                Servo_open_green_left();
            } else if(path == 5 || path == 9) {
                Servo_open_green_right();
            }
        } else if(aRxBuffer[0] == 51) {
            if(path == 3 || path == 7 || path == 11) {
                Servo_close();
            } else if(path == 5 || path == 9) {
                Servo_close();
            }
        }
        
        last_data = aRxBuffer[0];
        last_time = current_time;
    }

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
    
     while (1)//舵机测试
     {
    Servo_close();
    HAL_Delay(2000);
    Servo_open_green_right();
    HAL_Delay(1000);

    // Servo_close();
     }

    if (current_time - oled_prev_time >= 100) {  // 每100ms更新一次显示
        // 显示原始距离和滤波后的距离
        OLED_ShowString(1, 1, "Flt:");
        OLED_ShowString(1, 9, "Flt:");
        OLED_ShowString(2, 1, "Flt:");
        OLED_ShowString(2, 9, "Flt:");
        
        // 显示第一个传感器的原始值和滤波值
        OLED_ShowNum(1, 5, distances[0], 4);
        OLED_ShowNum(1, 13, fabs(distances[1]), 4);
        
        // 显示第二个传感器的原始值和滤波值
        OLED_ShowNum(2, 5, distances[2], 4);
        OLED_ShowNum(2, 13, distances[3], 4);
        

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
    OLED_ShowNum(3,6,fabsf(yaw),3);
    OLED_ShowChar(3,13,target_yaw >= 0 ? '+' : '-'); 
    OLED_ShowNum(3,14,fabsf(target_yaw),3);
    
    //OLED_ShowNum(4,1,path,2);  // 显示毫秒
    // OLED_ShowNum(4,4,time,4); OLED_ShowNum(4,10,time_start,4);
    // meandistances(distances);  
    // static bool rotation_test_done = false;

    // if (rotation_test_done==false)
    // {
    //   Rotate_90_Degrees(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, true);

    //   HAL_Delay(1000);

    //   Rotate_90_Degrees(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, false );

    //   rotation_test_done = true; 

    // }
    
    // while(1){
    // Servo_open_blue();
    // HAL_Delay(1000);
    // Servo_close();
    // HAL_Delay(1000);
    // }

    // Adjust_Motors_By_Side_Distances(MOTOR_1, MOTOR_4, MOTOR_2, MOTOR_3, raw_distances[0], raw_distances[2], 82.0f);


     switch (path) {
       case 0: {
         // 参数定义
         const float TARGET_DISTANCE = 135.0f;   // 调试，这个变量用于检测最终的目标距离
         const float DECEL_RANGE = 730.0f;      // 调试，这个变量用于设置减速区间范围
         const uint16_t ADJUST_DISTANCE = 250;  // 调试，这个变量用于在距离最终目标距离较近时的取消调校
         const uint8_t MIN_SPEED = 15;          // 调试，这个变量用于设置接近目标时的速度最小速度（靠近时）
         const uint8_t MAX_SPEED = 42;          // 调试，这个变量用于设置离目标较远时的速度
         const uint16_t DELAY_ADJUST = 8000;    // 调试，这个变量用于路径转换后的校准延时时间，需要确保进入垄  
         float current_distance = distances[1];
         if(time_enterpath_case0 == 0) {
             time_enterpath_case0 = HAL_GetTick();
         }
    
         // 速度计算逻辑
         uint8_t motor_speed = MAX_SPEED;  // 默认最大速度
    
         if (current_distance <= TARGET_DISTANCE && current_distance != 0 && (HAL_GetTick() - time_enterpath_case0 >= 3000)) {
             // 区域3：到达目标距离（≤80mm）
             motor_speed = MIN_SPEED;
             Rotate_90_Degrees(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, true );
             path += 1;
             PID_ResetAll(); // 重置所有PID控制器

         }
         else if (current_distance <= (TARGET_DISTANCE + DECEL_RANGE) /* && mean[1] <= (TARGET_DISTANCE + DECEL_RANGE) */) {
             // 区域2：减速区间（70~170mm）
             // 距离越近速度越慢，线性变化：170mm->60, 70mm->10
             float distance_from_target = current_distance - TARGET_DISTANCE;
             float ratio = (distance_from_target / DECEL_RANGE);
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
         if(HAL_GetTick() - time_enterpath_case0 >= DELAY_ADJUST && current_distance >= ADJUST_DISTANCE ){
             Adjust_Left_Motors_By_Distance(MOTOR_1, MOTOR_3, MOTOR_2, MOTOR_4, raw_distances[0], 50.0f);
         }
         OLED_ShowNum(4, 1, path, 2);
         OLED_ShowNum(4, 4, motor_speed, 2);
         break;
       
       }
       case 1: {
         // 参数定义
         const float TARGET_DISTANCE = 140.0f;   // 调试，这个变量用于检测最终的目标距离
         const float DECEL_RANGE = 700.0f;      // 调试，这个变量用于设置减速区间范围
         const uint16_t ADJUST_DISTANCE = 250;  // 调试，这个变量用于在距离最终目标距离较近时的取消调校
         const uint8_t MIN_SPEED = 15;          // 调试，这个变量用于设置接近目标时的速度最小速度（靠近时）
         const uint8_t MAX_SPEED = 42;          // 调试，这个变量用于设置离目标较远时的速度
         const uint16_t DELAY_ADJUST = 2700;    // 调试，这个变量用于路径转换后的校准延时时间，需要确保进入垄
    
         float current_distance = distances[1];
         if(time_enterpath_case1 == 0) {
             time_enterpath_case1 = HAL_GetTick();
         }
    
         // 速度计算逻辑
         uint8_t motor_speed = MAX_SPEED;  // 默认最大速度
    
         // 添加延迟判定
         static uint32_t reach_target_time = 0;
         if (reach_target_time == 0) {
             reach_target_time = HAL_GetTick();
         }
    
         if (current_distance <= TARGET_DISTANCE) {
             // 区域3：到达目标距离（≤130mm）
             motor_speed = MIN_SPEED;
             
             // 执行路径切换逻辑
             if(current_distance <= TARGET_DISTANCE && (HAL_GetTick() - reach_target_time >= 3000)) {
                Rotate_90_Degrees(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, false );
                 path += 1;
                 PID_ResetAll(); // 重置所有PID控制器
                 reach_target_time = 0; // 重置时间戳
             }
         }
         else if (current_distance <= (TARGET_DISTANCE + DECEL_RANGE)) {
             // 区域2：减速区间（130~830mm）
             float distance_from_target = current_distance - TARGET_DISTANCE;
             float ratio = (distance_from_target / DECEL_RANGE);
             motor_speed = MIN_SPEED + (uint8_t)((MAX_SPEED - MIN_SPEED) * ratio);
             motor_speed = CLAMP(motor_speed, MIN_SPEED, MAX_SPEED);
         }
         else {
             // 区域1：全速区间（>830mm）
             motor_speed = MAX_SPEED;
         }
    
         // 执行带平滑过渡的电机控制
         static uint8_t last_speed = 0;
         motor_speed = smooth_speed_transition(last_speed, motor_speed);
         last_speed = motor_speed;
    
         Motor_Straight(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, motor_speed, &yaw, &target_yaw);
        
         // 使用前后电机调整
         float avg_distance = distances[0];
         if(HAL_GetTick() - time_enterpath_case1 >= DELAY_ADJUST && current_distance >= ADJUST_DISTANCE ){
             Adjust_Left_Motors_By_Distance(MOTOR_1, MOTOR_4, MOTOR_2, MOTOR_3, raw_distances[0], 82.0f);
         }
    
         OLED_ShowNum(4, 4, motor_speed, 2);
         OLED_ShowNum(4, 1, path, 2);
         break;
       }
       case 2: {
         const uint32_t DELAY_ENTER = 200; //调试

         if (path_change!=2)
         {
           if ((distances[0]>=150 && path_change==0)||(distances[0]<=150 && path_change==1))
           {
             Motor_Straight(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, -33, &yaw, &target_yaw);
             // 使用右侧电机调整
             Adjust_Right_Motors_By_Distance(MOTOR_2, MOTOR_4, MOTOR_1, MOTOR_3, raw_distances[2], 50.0f);
           }else if (distances[0]<=190 && path_change==0)
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
           }else if (distances[0]>=180 && path_change==1)
           {
             if(flag){
               time_start = HAL_GetTick();
               flag = false;
             }
             uint32_t time = HAL_GetTick();
             if(time - time_start >= DELAY_ENTER ){
               path_change+=1;
               flag = true;
             }
           }
         }else{
           // 直接执行旋转和路径切换
           Rotate_90_Degrees(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, true );
           path_change = 0;
           flag = true;
           path +=1;
           PID_ResetAll();
         }
         OLED_ShowNum(4, 1, path, 2);
         break;
       }

       case 3: {
         // 参数定义
         const float TARGET_DISTANCE = 140.0f;   // 调试，这个变量用于检测最终的目标距离
         const float DECEL_RANGE = 700.0f;      // 调试，这个变量用于设置减速区间范围
         const uint16_t ADJUST_DISTANCE = 250;  // 调试，这个变量用于在距离最终目标距离较近时的取消调校
         const uint8_t MIN_SPEED = 15;          // 调试，这个变量用于设置接近目标时的速度最小速度（靠近时）
         const uint8_t MAX_SPEED = 42;          // 调试，这个变量用于设置离目标较远时的速度
         const uint16_t DELAY_ADJUST = 2700;    // 调试，这个变量用于路径转换后的校准延时时间，需要确保进入垄
    
         float current_distance = distances[3];
         if(time_enterpath_case3 == 0) {
             time_enterpath_case3 = HAL_GetTick();
         }
    
         // 速度计算逻辑
         uint8_t motor_speed = MAX_SPEED;  // 默认最大速度
    
         // 添加延迟判定
         static uint32_t reach_target_time = 0;
         if (reach_target_time == 0) {
             reach_target_time = HAL_GetTick();
         }
    
         if (current_distance <= TARGET_DISTANCE) {
             // 区域3：到达目标距离（≤130mm）
             motor_speed = MIN_SPEED;
             
             // 执行路径切换逻辑
             if(current_distance <= TARGET_DISTANCE && (HAL_GetTick() - reach_target_time >= 1000)) {
                Rotate_90_Degrees(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, false );
                 path += 1;
                 PID_ResetAll(); // 重置所有PID控制器
                 reach_target_time = 0; // 重置时间戳
             }
         }
         else if (current_distance <= (TARGET_DISTANCE + DECEL_RANGE)) {
             // 区域2：减速区间（130~830mm）
             float distance_from_target = current_distance - TARGET_DISTANCE;
             float ratio = (distance_from_target / DECEL_RANGE);
             motor_speed = MIN_SPEED + (uint8_t)((MAX_SPEED - MIN_SPEED) * ratio);
             motor_speed = CLAMP(motor_speed, MIN_SPEED, MAX_SPEED);
         }
         else {
             // 区域1：全速区间（>830mm）
             motor_speed = MAX_SPEED;
         }
    
         // 执行带平滑过渡的电机控制
         static uint8_t last_speed = 0;
         motor_speed = smooth_speed_transition(last_speed, motor_speed);
         last_speed = motor_speed;
    
         Motor_Straight(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, -motor_speed, &yaw, &target_yaw);
        
         // 使用前后电机调整
         float avg_distance = distances[0];
         if(HAL_GetTick() - time_enterpath_case3 >= DELAY_ADJUST && current_distance >= ADJUST_DISTANCE ){
             Adjust_Motors_By_Side_Distances(MOTOR_1, MOTOR_4, MOTOR_2, MOTOR_3, raw_distances[0], raw_distances[2], 82.0f);
         }
    
         OLED_ShowNum(4, 4, motor_speed, 2);
         OLED_ShowNum(4, 1, path, 2);
         break;
       }

       case 4: {
         const uint32_t DELAY_ENTER = 280; //调试

         if (path_change!=2)
         {
           if ((distances[2]>=150 && path_change==0)||(distances[2]<=150 && path_change==1))
           {
             Motor_Straight(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, -30, &yaw, &target_yaw);
             // 使用左侧电机调整
             Adjust_Left_Motors_By_Distance(MOTOR_1, MOTOR_3, MOTOR_2, MOTOR_4, raw_distances[0], 50.0f);
           }else if (distances[2]<=190 && path_change==0)
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
           }else if (distances[2]>=180 && path_change==1)
           {
             if(flag){
               time_start = HAL_GetTick();
               flag = false;
             }
             uint32_t time = HAL_GetTick();
             if(time - time_start >= DELAY_ENTER ){
               path_change+=1;
               flag = true;
             }
           }
         }else{
           // 直接执行旋转和路径切换
           Rotate_90_Degrees(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, true );
           path_change = 0;
           flag = true;
           path +=1;
           PID_ResetAll();
         }
         OLED_ShowNum(4, 1, path, 2);
         break;
       }

       case 5: {
         // 参数定义
         const float TARGET_DISTANCE = 140.0f;   // 调试，这个变量用于检测最终的目标距离
         const float DECEL_RANGE = 700.0f;      // 调试，这个变量用于设置减速区间范围
         const uint16_t ADJUST_DISTANCE = 250;  // 调试，这个变量用于在距离最终目标距离较近时的取消调校
         const uint8_t MIN_SPEED = 15;          // 调试，这个变量用于设置接近目标时的速度最小速度（靠近时）
         const uint8_t MAX_SPEED = 42;          // 调试，这个变量用于设置离目标较远时的速度
         const uint16_t DELAY_ADJUST = 2700;    // 调试，这个变量用于路径转换后的校准延时时间，需要确保进入垄
    
         float current_distance = distances[1];
         if(time_enterpath_case5 == 0) {
             time_enterpath_case5 = HAL_GetTick();
         }
    
         // 速度计算逻辑
         uint8_t motor_speed = MAX_SPEED;  // 默认最大速度
    
         // 添加延迟判定
         static uint32_t reach_target_time = 0;
         if (reach_target_time == 0) {
             reach_target_time = HAL_GetTick();
         }
    
         if (current_distance <= TARGET_DISTANCE) {
             // 区域3：到达目标距离（≤130mm）
             motor_speed = MIN_SPEED;
             
             // 执行路径切换逻辑
             if(current_distance <= TARGET_DISTANCE && (HAL_GetTick() - reach_target_time >= 1000)) {
                Rotate_90_Degrees(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, false );
                 path += 1;
                 PID_ResetAll(); // 重置所有PID控制器
                 reach_target_time = 0; // 重置时间戳
             }
         }
         else if (current_distance <= (TARGET_DISTANCE + DECEL_RANGE)) {
             // 区域2：减速区间（130~830mm）
             float distance_from_target = current_distance - TARGET_DISTANCE;
             float ratio = (distance_from_target / DECEL_RANGE);
             motor_speed = MIN_SPEED + (uint8_t)((MAX_SPEED - MIN_SPEED) * ratio);
             motor_speed = CLAMP(motor_speed, MIN_SPEED, MAX_SPEED);
         }
         else {
             // 区域1：全速区间（>830mm）
             motor_speed = MAX_SPEED;
         }
    
         // 执行带平滑过渡的电机控制
         static uint8_t last_speed = 0;
         motor_speed = smooth_speed_transition(last_speed, motor_speed);
         last_speed = motor_speed;
    
         Motor_Straight(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, motor_speed, &yaw, &target_yaw);
        
         // 使用前后电机调整
         float avg_distance = distances[0];
         if(HAL_GetTick() - time_enterpath_case5 >= DELAY_ADJUST && current_distance >= ADJUST_DISTANCE ){
             Adjust_Motors_By_Side_Distances(MOTOR_1, MOTOR_4, MOTOR_2, MOTOR_3, raw_distances[0], raw_distances[2], 82.0f);
         }
    
         OLED_ShowNum(4, 4, motor_speed, 2);
         OLED_ShowNum(4, 1, path, 2);
         break;
       }

       case 6: {
         const uint32_t DELAY_ENTER = 200; //调试

         if (path_change!=2)
         {
           if ((distances[0]>=150 && path_change==0)||(distances[0]<=150 && path_change==1))
           {
             Motor_Straight(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, -30, &yaw, &target_yaw);
             // 使用右侧电机调整
             Adjust_Right_Motors_By_Distance(MOTOR_2, MOTOR_4, MOTOR_1, MOTOR_3, raw_distances[2], 50.0f);
           }else if (distances[0]<=190 && path_change==0)
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
           }else if (distances[0]>=180 && path_change==1)
           {
             if(flag){
               time_start = HAL_GetTick();
               flag = false;
             }
             uint32_t time = HAL_GetTick();
             if(time - time_start >= DELAY_ENTER ){
               path_change+=1;
               flag = true;
             }
           }
         }else{
           // 直接执行旋转和路径切换
           Rotate_90_Degrees(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, true );
           path_change = 0;
           flag = true;
           path +=1;
           PID_ResetAll();
         }
         OLED_ShowNum(4, 1, path, 2);
         break;
       }

       case 7: {
         // 参数定义
         const float TARGET_DISTANCE = 140.0f;   // 调试，这个变量用于检测最终的目标距离
         const float DECEL_RANGE = 700.0f;      // 调试，这个变量用于设置减速区间范围
         const uint16_t ADJUST_DISTANCE = 250;  // 调试，这个变量用于在距离最终目标距离较近时的取消调校
         const uint8_t MIN_SPEED = 15;          // 调试，这个变量用于设置接近目标时的速度最小速度（靠近时）
         const uint8_t MAX_SPEED = 42;          // 调试，这个变量用于设置离目标较远时的速度
         const uint16_t DELAY_ADJUST = 2700;    // 调试，这个变量用于路径转换后的校准延时时间，需要确保进入垄
    
         float current_distance = distances[3];
         if(time_enterpath_case7 == 0) {
             time_enterpath_case7 = HAL_GetTick();
         }
    
         // 速度计算逻辑
         uint8_t motor_speed = MAX_SPEED;  // 默认最大速度
    
         // 添加延迟判定
         static uint32_t reach_target_time = 0;
         if (reach_target_time == 0) {
             reach_target_time = HAL_GetTick();
         }
    
         if (current_distance <= TARGET_DISTANCE) {
             // 区域3：到达目标距离（≤130mm）
             motor_speed = MIN_SPEED;
             
             // 执行路径切换逻辑
             if(current_distance <= TARGET_DISTANCE && (HAL_GetTick() - reach_target_time >= 3000)) {
                Rotate_90_Degrees(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, false );
                 path += 1;
                 PID_ResetAll(); // 重置所有PID控制器
                 reach_target_time = 0; // 重置时间戳
             }
         }
         else if (current_distance <= (TARGET_DISTANCE + DECEL_RANGE)) {
             // 区域2：减速区间（130~830mm）
             float distance_from_target = current_distance - TARGET_DISTANCE;
             float ratio = (distance_from_target / DECEL_RANGE);
             motor_speed = MIN_SPEED + (uint8_t)((MAX_SPEED - MIN_SPEED) * ratio);
             motor_speed = CLAMP(motor_speed, MIN_SPEED, MAX_SPEED);
         }
         else {
             // 区域1：全速区间（>830mm）
             motor_speed = MAX_SPEED;
         }
    
         // 执行带平滑过渡的电机控制
         static uint8_t last_speed = 0;
         motor_speed = smooth_speed_transition(last_speed, motor_speed);
         last_speed = motor_speed;
    
         Motor_Straight(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, -motor_speed, &yaw, &target_yaw);
        
         // 使用前后电机调整
         float avg_distance = distances[0];
         if(HAL_GetTick() - time_enterpath_case7 >= DELAY_ADJUST && current_distance >= ADJUST_DISTANCE ){
             Adjust_Motors_By_Side_Distances(MOTOR_1, MOTOR_4, MOTOR_2, MOTOR_3, raw_distances[0], raw_distances[2], 82.0f);
         }
    
         OLED_ShowNum(4, 4, motor_speed, 2);
         OLED_ShowNum(4, 1, path, 2);
         break;
       }

       case 8: {
         const uint32_t DELAY_ENTER = 280; //调试

         if (path_change!=2)
         {
           if ((distances[2]>=150 && path_change==0)||(distances[2]<=150 && path_change==1))
           {
             Motor_Straight(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, -30, &yaw, &target_yaw);
             // 使用左侧电机调整
             Adjust_Left_Motors_By_Distance(MOTOR_1, MOTOR_3, MOTOR_2, MOTOR_4, raw_distances[0], 50.0f);
           }else if (distances[2]<=190 && path_change==0)
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
           }else if (distances[2]>=180 && path_change==1)
           {
             if(flag){
               time_start = HAL_GetTick();
               flag = false;
             }
             uint32_t time = HAL_GetTick();
             if(time - time_start >= DELAY_ENTER ){
               path_change+=1;
               flag = true;
             }
           }
         }else{
           // 直接执行旋转和路径切换
           Rotate_90_Degrees(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, true );
           path_change = 0;
           flag = true;
           path +=1;
           PID_ResetAll();
         }
         OLED_ShowNum(4, 1, path, 2);
         break;
       }

       case 9: {
         // 参数定义
         const float TARGET_DISTANCE = 140.0f;   // 调试，这个变量用于检测最终的目标距离
         const float DECEL_RANGE = 700.0f;      // 调试，这个变量用于设置减速区间范围
         const uint16_t ADJUST_DISTANCE = 250;  // 调试，这个变量用于在距离最终目标距离较近时的取消调校
         const uint8_t MIN_SPEED = 15;          // 调试，这个变量用于设置接近目标时的速度最小速度（靠近时）
         const uint8_t MAX_SPEED = 42;          // 调试，这个变量用于设置离目标较远时的速度
         const uint16_t DELAY_ADJUST = 2700;    // 调试，这个变量用于路径转换后的校准延时时间，需要确保进入垄
    
         float current_distance = distances[1];
         if(time_enterpath_case9 == 0) {
             time_enterpath_case9 = HAL_GetTick();
         }
    
         // 速度计算逻辑
         uint8_t motor_speed = MAX_SPEED;  // 默认最大速度
    
         // 添加延迟判定
         static uint32_t reach_target_time = 0;
         if (reach_target_time == 0) {
             reach_target_time = HAL_GetTick();
         }
    
         if (current_distance <= TARGET_DISTANCE) {
             // 区域3：到达目标距离（≤130mm）
             motor_speed = MIN_SPEED;
             
             // 执行路径切换逻辑
             if(current_distance <= TARGET_DISTANCE && (HAL_GetTick() - reach_target_time >= 3000)) {
                Rotate_90_Degrees(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, false );
                 path += 1;
                 PID_ResetAll(); // 重置所有PID控制器
                 reach_target_time = 0; // 重置时间戳
             }
         }
         else if (current_distance <= (TARGET_DISTANCE + DECEL_RANGE)) {
             // 区域2：减速区间（130~830mm）
             float distance_from_target = current_distance - TARGET_DISTANCE;
             float ratio = (distance_from_target / DECEL_RANGE);
             motor_speed = MIN_SPEED + (uint8_t)((MAX_SPEED - MIN_SPEED) * ratio);
             motor_speed = CLAMP(motor_speed, MIN_SPEED, MAX_SPEED);
         }
         else {
             // 区域1：全速区间（>830mm）
             motor_speed = MAX_SPEED;
         }
    
         // 执行带平滑过渡的电机控制
         static uint8_t last_speed = 0;
         motor_speed = smooth_speed_transition(last_speed, motor_speed);
         last_speed = motor_speed;
    
         Motor_Straight(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, motor_speed, &yaw, &target_yaw);
        
         // 使用前后电机调整
         float avg_distance = distances[0];
         if(HAL_GetTick() - time_enterpath_case9 >= DELAY_ADJUST && current_distance >= ADJUST_DISTANCE + 100 ){
             Adjust_Motors_By_Side_Distances(MOTOR_1, MOTOR_4, MOTOR_2, MOTOR_3, raw_distances[0], raw_distances[2], 82.0f);
         }
    
         OLED_ShowNum(4, 4, motor_speed, 2);
         OLED_ShowNum(4, 1, path, 2);
         break;
       }

       case 10: {
         const uint32_t DELAY_ENTER = 220; //调试

         if (path_change!=2)
         {
           if ((distances[0]>=150 && path_change==0)||(distances[0]<=150 && path_change==1))
           {
             Motor_Straight(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, -30, &yaw, &target_yaw);
             // 使用右侧电机调整
             Adjust_Right_Motors_By_Distance(MOTOR_2, MOTOR_4, MOTOR_1, MOTOR_3, raw_distances[2], 50.0f);
           }else if (distances[0]<=190 && path_change==0)
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
           }else if (distances[0]>=180 && path_change==1)
           {
             if(flag){
               time_start = HAL_GetTick();
               flag = false;
             }
             uint32_t time = HAL_GetTick();
             if(time - time_start >= DELAY_ENTER){
               path_change+=1;
               flag = true;
             }
           }
         }else{
           // 直接执行旋转和路径切换
           Rotate_90_Degrees(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, true );
           path_change = 0;
           flag = true;
           path +=1;
           PID_ResetAll();
         }
         OLED_ShowNum(4, 1, path, 2);
         break;
       }

       case 11: {
         // 参数定义
         const float TARGET_DISTANCE = 140.0f;   // 调试，这个变量用于检测最终的目标距离
         const float DECEL_RANGE = 700.0f;      // 调试，这个变量用于设置减速区间范围
         const uint16_t ADJUST_DISTANCE = 250;  // 调试，这个变量用于在距离最终目标距离较近时的取消调校
         const uint8_t MIN_SPEED = 15;          // 调试，这个变量用于设置接近目标时的速度最小速度（靠近时）
         const uint8_t MAX_SPEED = 42;          // 调试，这个变量用于设置离目标较远时的速度
         const uint16_t DELAY_ADJUST = 2700;    // 调试，这个变量用于路径转换后的校准延时时间，需要确保进入垄
    
         float current_distance = distances[3];
         if(time_enterpath_case11 == 0) {
             time_enterpath_case11 = HAL_GetTick();
         }
    
         // 速度计算逻辑
         uint8_t motor_speed = MAX_SPEED;  // 默认最大速度
    
         // 添加延迟判定
         static uint32_t reach_target_time = 0;
         if (reach_target_time == 0) {
             reach_target_time = HAL_GetTick();
         }
    
         if (current_distance <= TARGET_DISTANCE) {
             // 区域3：到达目标距离（≤130mm）
             motor_speed = MIN_SPEED;
             
             // 执行路径切换逻辑
             if(current_distance <= TARGET_DISTANCE && (HAL_GetTick() - reach_target_time >= 3000)) {
                Rotate_90_Degrees(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, false );
                 path += 1;
                 PID_ResetAll(); // 重置所有PID控制器
                 reach_target_time = 0; // 重置时间戳
             }
         }
         else if (current_distance <= (TARGET_DISTANCE + DECEL_RANGE)) {
             // 区域2：减速区间（130~830mm）
             float distance_from_target = current_distance - TARGET_DISTANCE;
             float ratio = (distance_from_target / DECEL_RANGE);
             motor_speed = MIN_SPEED + (uint8_t)((MAX_SPEED - MIN_SPEED) * ratio);
             motor_speed = CLAMP(motor_speed, MIN_SPEED, MAX_SPEED);
         }
         else {
             // 区域1：全速区间（>830mm）
             motor_speed = MAX_SPEED;
         }
    
         // 执行带平滑过渡的电机控制
         static uint8_t last_speed = 0;
         motor_speed = smooth_speed_transition(last_speed, motor_speed);
         last_speed = motor_speed;
    
         Motor_Straight(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, -motor_speed, &yaw, &target_yaw);
        
         // 使用前后电机调整
         float avg_distance = distances[0];
         if(HAL_GetTick() - time_enterpath_case11 >= DELAY_ADJUST && current_distance >= ADJUST_DISTANCE ){
             Adjust_Motors_By_Side_Distances(MOTOR_1, MOTOR_4, MOTOR_2, MOTOR_3, raw_distances[0], raw_distances[2], 82.0f);
         }
    
         OLED_ShowNum(4, 4, motor_speed, 2);
         OLED_ShowNum(4, 1, path, 2);
         break;
       }

       case 12: {
         static uint32_t start_backward_time = 0;
         
         if (start_backward_time == 0) {
             start_backward_time = HAL_GetTick();
         }
         
         uint32_t current_time = HAL_GetTick();
         if (current_time - start_backward_time < 5000) {  // 5秒内后退
             Motor_Straight(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, -30, &yaw, &target_yaw);
             // 使用左侧电机调整
             Adjust_Left_Motors_By_Distance(MOTOR_1, MOTOR_3, MOTOR_2, MOTOR_4, raw_distances[0], 50.0f);
         } else {
             // 5秒后停车
             Motor_SetSpeed(MOTOR_1, 0);
             Motor_SetSpeed(MOTOR_2, 0);
             Motor_SetSpeed(MOTOR_3, 0);
             Motor_SetSpeed(MOTOR_4, 0);
         }
         
         OLED_ShowNum(4, 1, path, 2);
         break;
       }
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
