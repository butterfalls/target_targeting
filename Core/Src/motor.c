#include "motor.h"
#include "usart.h"
#include "mpu6050.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "oled.h"

// 定义圆周率
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Private variables ---------------------------------------------------------*/
float target_yaw = 0.0f;
float yaw = 0.0f; 

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
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
    HAL_TIM_Encoder_Start(encoder_tim, TIM_CHANNEL_1|TIM_CHANNEL_2);
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

void Motor_Rightward(Motor_ID id1, Motor_ID id2, Motor_ID id3, Motor_ID id4, int16_t speed, float* yaw, float* target_yaw) {
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - prev_time) / 1000.0f;

    if (dt <= 0.001f) {
        dt = 0.001f;  // 最小时间差为1ms
    }
    prev_time = current_time;

    // 获取编码器值
    int32_t enc1 = Motor_GetEncoder(id1);
    int32_t enc2 = -Motor_GetEncoder(id2);
    int32_t enc3 = Motor_GetEncoder(id3);
    int32_t enc4 = -Motor_GetEncoder(id4);

    // 获取当前偏航角
    float pitch, roll, current_yaw;
    if (MPU6050_DMP_Get_Data(&pitch, &roll, &current_yaw) != 0) {
        Motor_SetSpeed(id1, 0);
        Motor_SetSpeed(id2, 0);
        Motor_SetSpeed(id3, 0);
        Motor_SetSpeed(id4, 0);
        return;
    }
    *yaw = current_yaw;
        
    float yaw_error = *target_yaw - *yaw;
    if (yaw_error > 180) yaw_error -= 360;
    else if (yaw_error < -180) yaw_error += 360;

    // 计算编码器误差
    int32_t front_error = enc1 - enc2;  // 前侧轮子同步
    int32_t rear_error = enc3 - enc4;   // 后侧轮子同步
    int32_t position_error = (front_error + rear_error) / 2;  // 左右两侧同步

    // 速度分配 - 修正后的分配方式
    float base_speed = speed;
    
    // 限制PID输出的最大值，防止过度修正
    float max_pid_output = base_speed * 0.3f;  // 降低PID输出最大值为基准速度的30%
    
    // 计算偏航角PID输出
    float yaw_pid_output = 0.0f;
    if (fabs(yaw_error) > 1.0f) {
        yaw_pid_output = PID_Calculate(&pid_yaw, yaw_error, dt);
        yaw_pid_output = fmaxf(fminf(yaw_pid_output, max_pid_output*10), -max_pid_output*10);
    } else {
        // 误差小于1度时，重置PID控制器
        PID_Reset(&pid_yaw);
    }

    // OLED_ShowNum(4,1,yaw_error,3);
    // OLED_ShowNum(1,1,yaw_pid_output,5);
    
    // 计算前后轮组的PID输出
    float front_pid_output = PID_Calculate(&pid_front, front_error, dt);
    float rear_pid_output = PID_Calculate(&pid_rear, rear_error, dt);
    float position_pid_output = PID_Calculate(&pid_position, position_error - yaw_pid_output, dt);
    
    // 限制PID输出
    front_pid_output = fmaxf(fminf(front_pid_output, max_pid_output), -max_pid_output);
    rear_pid_output = fmaxf(fminf(rear_pid_output, max_pid_output), -max_pid_output);
    position_pid_output = fmaxf(fminf(position_pid_output, max_pid_output), -max_pid_output);
    
    // 前侧轮子 - 向内运动
    float speed1 = -(base_speed - front_pid_output - position_pid_output - yaw_pid_output);  // 左前
    float speed2 = (base_speed + front_pid_output + position_pid_output + yaw_pid_output);   // 右后
    
    // 后侧轮子 - 向内运动
    float speed3 = -(base_speed + rear_pid_output + position_pid_output + yaw_pid_output);   // 左后
    float speed4 = (base_speed - rear_pid_output - position_pid_output - yaw_pid_output);    // 右前

    // 限幅
    speed1 = fmaxf(fminf(speed1, 100.0f), -100.0f);
    speed2 = fmaxf(fminf(speed2, 100.0f), -100.0f);
    speed3 = fmaxf(fminf(speed3, 100.0f), -100.0f);
    speed4 = fmaxf(fminf(speed4, 100.0f), -100.0f);

    // 设置电机速度
    Motor_SetSpeed(id1, speed1);
    Motor_SetSpeed(id2, speed2);
    Motor_SetSpeed(id3, speed3);
    Motor_SetSpeed(id4, speed4);
}

void Motor_Straight(Motor_ID id1, Motor_ID id2, Motor_ID id3, Motor_ID id4, int16_t speed, float* yaw, float* target_yaw) {
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - prev_time) / 1000.0f;
    
    // 添加时间差保护
    if (dt <= 0.001f) {
        dt = 0.001f;  // 最小时间差为1ms
    }
    prev_time = current_time;

    // 获取编码器值
    int32_t enc1 = Motor_GetEncoder(id1);
    int32_t enc2 = -Motor_GetEncoder(id2);
    int32_t enc3 = Motor_GetEncoder(id3);
    int32_t enc4 = -Motor_GetEncoder(id4);

    // 获取当前偏航角
    float pitch, roll, current_yaw;
    if (MPU6050_DMP_Get_Data(&pitch, &roll, &current_yaw) != 0) {
        Motor_SetSpeed(id1, 0);
        Motor_SetSpeed(id2, 0);
        Motor_SetSpeed(id3, 0);
        Motor_SetSpeed(id4, 0);
        return;
    }
    *yaw = current_yaw;

    // 计算偏航角误差
    float yaw_error = *target_yaw - *yaw;
    if (yaw_error > 180) yaw_error -= 360;
    else if (yaw_error < -180) yaw_error += 360;

    // 计算编码器误差
    int32_t left_error = enc1 - enc3;  // 左侧轮子同步
    int32_t right_error = enc2 - enc4;  // 右侧轮子同步
    int32_t position_error = (left_error + right_error) / 2;  // 左右两侧同步

    // 速度分配 - 修正后的分配方式
    float base_speed = speed;
    
    // 限制PID输出的最大值，防止过度修正
    float max_pid_output = base_speed * 0.3f;  // 降低PID输出最大值为基准速度的30%
    
    // 计算偏航角PID输出
    float yaw_pid_output = 0.0f;
    if (fabs(yaw_error) > 1.0f) {
        yaw_pid_output = PID_Calculate(&pid_yaw, yaw_error, dt);
        yaw_pid_output = fmaxf(fminf(yaw_pid_output, max_pid_output*10), -max_pid_output*10);
    } else {
        // 误差小于1度时，重置PID控制器
        PID_Reset(&pid_yaw);
    }
    
    // 计算左右轮组的PID输出
    float left_pid_output = PID_Calculate(&pid_front, left_error, dt);
    float right_pid_output = PID_Calculate(&pid_rear, right_error, dt);
    float position_pid_output = PID_Calculate(&pid_position, position_error - yaw_pid_output, dt);
    
    // 限制PID输出
    left_pid_output = fmaxf(fminf(left_pid_output, max_pid_output), -max_pid_output);
    right_pid_output = fmaxf(fminf(right_pid_output, max_pid_output), -max_pid_output);
    position_pid_output = fmaxf(fminf(position_pid_output, max_pid_output), -max_pid_output);
    
    // 左侧轮子 - 正转
    float speed1 = -(base_speed - left_pid_output - position_pid_output - yaw_pid_output);  // 左前
    float speed3 = (base_speed - left_pid_output - position_pid_output - yaw_pid_output);   // 左后
    
    // 右侧轮子 - 反转
    float speed2 = (base_speed + right_pid_output + position_pid_output + yaw_pid_output);  // 右前
    float speed4 = -(base_speed + right_pid_output + position_pid_output + yaw_pid_output); // 右后

    // 限幅
    speed1 = fmaxf(fminf(speed1, 100.0f), -100.0f);
    speed2 = fmaxf(fminf(speed2, 100.0f), -100.0f);
    speed3 = fmaxf(fminf(speed3, 100.0f), -100.0f);
    speed4 = fmaxf(fminf(speed4, 100.0f), -100.0f);

    // 设置电机速度
    Motor_SetSpeed(id1, speed1);
    Motor_SetSpeed(id2, speed2);
    Motor_SetSpeed(id3, speed3);
    Motor_SetSpeed(id4, speed4);
}

void straight_us100(float distance, float* yaw, float* target_yaw)
{
    float distance_forward = distance;
    
    if (distance_forward <= 0) {
        distance_forward = distance;
    } else {
        distance = distance_forward;
    }
    
    int16_t speed = 0;
    if (distance_forward > 1000) { 
        speed = 80;
    } else if (distance_forward > 500) { 
        speed = 30;
    } else if (distance_forward > 200) {
        speed = 15;
    } else { 
        speed = 0;
    }
    //Motor_Rightward()
}

void Update_Target_Yaw(float* yaw, float* target_yaw) 
/*要求：
0和1改为右侧两个超声波，2改为前向超声波
*/
{
    static float prev_distances[3] = {0.0f, 0.0f, 0.0f};  // 存储上一次的超声波距离
    static bool first_measurement = true;            // 是否是第一次测量
    float current_distances[4];
    US100_GetAllValidDistances(current_distances);                      // 当前超声波距离
    float angles[2];                                 // 计算出的角度
    float valid_angles[2];                           // 有效的角度值
    int valid_count = 0;                             // 有效角度计数
    
    // 如果是第一次测量，只记录距离
    if (first_measurement) {
        prev_distances[0] = current_distances[0];//把这两个超声波改成右侧的两个
        prev_distances[1] = current_distances[1];
        prev_distances[2] = current_distances[2];//前向超声波
        first_measurement = false;
        return;
    }    
    // 计算移动距离（使用前向超声波传感器测得值的差值）
    float move_distance = (current_distances[2] - prev_distances[2]);
    
    // 如果移动距离太小，不进行角度计算
    if (move_distance < 50.0f) {  // 5cm作为最小移动距离阈值
        return;
    }
    
    for (int i = 0; i < 2; i++) {
        float delta_distance = current_distances[i] - prev_distances[i];
        if (fabsf(delta_distance) > 0.1f) {  // 避免除以接近0的值
            angles[i] = atanf(delta_distance/move_distance) * 180.0f / M_PI;
            valid_angles[valid_count++] = angles[i];
        }
    }
    
    // 更新上一次的距离值
    prev_distances[0] = current_distances[0];
    prev_distances[1] = current_distances[1];
    
    // 如果有有效的角度值
    if (valid_count > 0) {
        float angle_diff = 0.0f;
        if (valid_count == 2) {
            angle_diff = fabsf(valid_angles[0] - valid_angles[1]);
            angle_diff = angle_diff/fabs(valid_angles[0]);
        
            if (angle_diff < 0.1f && angle_diff > -0.1f) {
                float avg_angle = 0.0f;
                for (int i = 0; i < valid_count; i++) {
                    avg_angle += valid_angles[i];
                }
                avg_angle /= valid_count;
                
                // 更新目标偏航角
                *target_yaw -= avg_angle;
                
                // 确保目标偏航角在-180到180度之间
                if (*target_yaw > 180.0f) {
                    *target_yaw -= 360.0f;
                } else if (*target_yaw < -180.0f) {
                    *target_yaw += 360.0f;
                }
            }
        }
    }
}

void Adjust_Speed_By_Side_Distance(Motor_ID id1, Motor_ID id2, int16_t base_speed, float side_distance, float target_distance)
{
    // 计算距离误差
    float distance_error = side_distance - target_distance;
    
    // 定义调整参数
    const float kp = 0.5f;  // 比例系数，可以根据实际情况调整
    const float max_adjustment = 30.0f;  // 最大速度调整量
    
    // 计算速度调整量
    float speed_adjustment = kp * distance_error;
    
    // 限制速度调整量
    if (speed_adjustment > max_adjustment) {
        speed_adjustment = max_adjustment;
    } else if (speed_adjustment < -max_adjustment) {
        speed_adjustment = -max_adjustment;
    }
    
    // 根据距离误差调整速度
    float speed1, speed2;
    
    if (distance_error < 0) {  // 距离过近
        // 靠近的一侧加速
        speed1 = base_speed + speed_adjustment;  // 靠近的一侧
        speed2 = base_speed;                     // 远离的一侧
    } else {  // 距离过远
        // 远离的一侧加速
        speed1 = base_speed;                     // 靠近的一侧
        speed2 = base_speed + speed_adjustment;  // 远离的一侧
    }
    
    // 限幅
    speed1 = fmaxf(fminf(speed1, 100.0f), -100.0f);
    speed2 = fmaxf(fminf(speed2, 100.0f), -100.0f);
    
    // 设置电机速度
    Motor_SetSpeed(id1, speed1);
    Motor_SetSpeed(id2, speed2);
}

