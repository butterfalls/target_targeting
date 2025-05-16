#include "motor.h"
#include "usart.h"
#include "mpu6050.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "oled.h"
#include "tim.h"
#include "us100_uart.h"

// 定义圆周率
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define PIDController pid_rotate = {3.0f, 0.1f, 0.5f, 0, 0}; // 需调试参数


/* Private variables ---------------------------------------------------------*/
float target_yaw = 0.0f;
float pitchstart=0.0f , rollopen=0.0f ,yaw = 0.0f; 

// 添加静态变量用于存储上一次的计数器值
static uint32_t prev_counter = 0;

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

    uint32_t duty = (speed * 9999) / 100;
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

// 添加时间相关函数
uint32_t Get_Time_Difference(void)
{
    uint32_t current_counter = __HAL_TIM_GET_COUNTER(&htim6);
    uint32_t diff;
    
    if (current_counter >= prev_counter) {
        diff = current_counter - prev_counter;
    } else {
        diff = (0xFFFFFFFF - prev_counter) + current_counter + 1;
    }
    
    prev_counter = current_counter;
    return diff ; 
}

void Reset_Timer(void)
{
    prev_counter = __HAL_TIM_GET_COUNTER(&htim6);
}

void Motor_Rightward(Motor_ID id1, Motor_ID id2, Motor_ID id3, Motor_ID id4, int16_t speed, float* yaw, float* target_yaw) {
    // 使用 HAL_GetTick 计算时间差
    static uint32_t prev_tick = 0;
    uint32_t current_tick = HAL_GetTick();
    float dt = (current_tick - prev_tick) / 1000.0f;  // 转换为秒
    prev_tick = current_tick;

    if (dt <= 0.001f) {
        dt = 0.001f;  // 最小时间差为1ms
    }

    // 静态变量存储上一次的编码器值
    static int32_t prev_enc1 = 0, prev_enc2 = 0, prev_enc3 = 0, prev_enc4 = 0;

    // 获取当前编码器值
    int32_t enc1 = Motor_GetEncoder(id1);
    int32_t enc2 = -Motor_GetEncoder(id2);
    int32_t enc3 = -Motor_GetEncoder(id3);
    int32_t enc4 = Motor_GetEncoder(id4);

    // 计算编码器速度（单位时间内的变化量）
    float speed1 = (enc1 - prev_enc1) / dt;
    float speed2 = (enc2 - prev_enc2) / dt;
    float speed3 = (enc3 - prev_enc3) / dt;
    float speed4 = (enc4 - prev_enc4) / dt;

    // 显示速度，分别显示正负号和4位数字
    // OLED_ShowChar(1,1,speed1 >= 0 ? '+' : '-');  // 左前符号
    // OLED_ShowNum(1,2,(int16_t)fabsf(speed1),4);  // 左前数字
    // OLED_ShowChar(1,9,speed2 >= 0 ? '+' : '-');  // 右后符号
    // OLED_ShowNum(1,10,(int16_t)fabsf(speed2),4);  // 右后数字
    // OLED_ShowChar(2,1,speed3 >= 0 ? '+' : '-');  // 左后符号
    // OLED_ShowNum(2,2,(int16_t)fabsf(speed3),4);  // 左后数字
    // OLED_ShowChar(2,9,speed4 >= 0 ? '+' : '-');  // 右前符号
    // OLED_ShowNum(2,10,(int16_t)fabsf(speed4),4);  // 右前数字

    // 更新上一次的编码器值
    prev_enc1 = enc1;
    prev_enc2 = enc2;
    prev_enc3 = enc3;
    prev_enc4 = enc4;

    // 获取当前偏航角
    float pitch, roll, current_yaw;
    if (MPU6050_DMP_Get_Data(&pitch, &roll, &current_yaw) != 0) {
        // Motor_SetSpeed(id1, 0);
        // Motor_SetSpeed(id2, 0);
        // Motor_SetSpeed(id3, 0);
        // Motor_SetSpeed(id4, 0);
        return;
    }
    *yaw = current_yaw;
        
    float yaw_error = *target_yaw - *yaw;
    if (yaw_error > 180) yaw_error -= 360;
    else if (yaw_error < -180) yaw_error += 360;

    // 计算速度误差
    float front_speed_error = speed1 - speed4;  // 前轮组速度同步（左前-右前）
    float rear_speed_error = speed2 - speed3;   // 后轮组速度同步（右后-左后）
    float position_speed_error = (front_speed_error - rear_speed_error) / 2;  // 前后轮组速度同步

    // 速度分配 - 修正后的分配方式
    float base_speed = speed;
    
    // 限制PID输出的最大值，防止过度修正
    float max_pid_output = base_speed * 0.3f;  // 降低PID输出最大值为基准速度的30%
    
    // 计算偏航角PID输出
    float yaw_pid_output = 0.0f;
    if (fabs(yaw_error) > 0.5f) {
        yaw_pid_output = PID_Calculate(&pid_yaw, yaw_error, dt);
        // yaw_pid_output = fmaxf(fminf(yaw_pid_output, max_pid_output*1), -max_pid_output*1);
    } else {
        // 误差小于1度时，重置PID控制器
        // PID_Reset(&pid_yaw);
    }
    
    // 计算前后轮组的速度PID输出
    float front_pid_output = PID_Calculate(&pid_front, front_speed_error, dt);
    float rear_pid_output = PID_Calculate(&pid_rear, rear_speed_error, dt);
    float position_pid_output = PID_Calculate(&pid_position, position_speed_error + yaw_pid_output, dt);
    
    // 限制PID输出
    // front_pid_output = fmaxf(fminf(front_pid_output, max_pid_output*1), -max_pid_output*1);
    // rear_pid_output = fmaxf(fminf(rear_pid_output, max_pid_output*1), -max_pid_output*1);
    // position_pid_output = fmaxf(fminf(position_pid_output, max_pid_output*1), -max_pid_output*1);
    
    // 前轮组 - 左前右前同步
    float motor_speed1 = -(base_speed - front_pid_output - position_pid_output - yaw_pid_output);  // 左前
    float motor_speed4 = (base_speed + front_pid_output - position_pid_output - yaw_pid_output);   // 右前
    
    // 后轮组 - 左后右后同步
    float motor_speed2 = (base_speed - rear_pid_output + position_pid_output + yaw_pid_output);   // 右后
    float motor_speed3 = -(base_speed + rear_pid_output + position_pid_output + yaw_pid_output);  // 左后

    // 限幅
    motor_speed1 = fmaxf(fminf(motor_speed1, 100.0f), -100.0f);
    motor_speed2 = fmaxf(fminf(motor_speed2, 100.0f), -100.0f);
    motor_speed3 = fmaxf(fminf(motor_speed3, 100.0f), -100.0f);
    motor_speed4 = fmaxf(fminf(motor_speed4, 100.0f), -100.0f);

    // 设置电机速度
    Motor_SetSpeed(id1, motor_speed1);
    Motor_SetSpeed(id2, motor_speed2);
    Motor_SetSpeed(id3, motor_speed3);
    Motor_SetSpeed(id4, motor_speed4);
}

void Motor_Straight(Motor_ID id1, Motor_ID id2, Motor_ID id3, Motor_ID id4, int16_t speed, float* yaw, float* target_yaw) {
    // 使用 HAL_GetTick 计算时间差
    static uint32_t prev_tick = 0;
    uint32_t current_tick = HAL_GetTick();
    float dt = (current_tick - prev_tick) / 1000.0f;  // 转换为秒
    prev_tick = current_tick;
        
    // 添加时间差保护
    if (dt <= 0.001f) {
        dt = 0.001f;  // 最小时间差为1ms
    }

    // 静态变量存储上一次的编码器值
    static int32_t prev_enc1 = 0, prev_enc2 = 0, prev_enc3 = 0, prev_enc4 = 0;

    // 获取当前编码器值
    int32_t enc1 = Motor_GetEncoder(id1);
    int32_t enc2 = -Motor_GetEncoder(id2);
    int32_t enc3 = Motor_GetEncoder(id3);
    int32_t enc4 = -Motor_GetEncoder(id4);

    // 计算编码器速度（单位时间内的变化量）
    float speed1 = (enc1 - prev_enc1) / dt;
    float speed2 = (enc2 - prev_enc2) / dt;
    float speed3 = (enc3 - prev_enc3) / dt;
    float speed4 = (enc4 - prev_enc4) / dt;

    // 显示速度，分别显示正负号和4位数字
    // OLED_ShowChar(1,1,speed1 >= 0 ? '+' : '-');  // 左前符号
    // OLED_ShowNum(1,2,(int16_t)fabsf(speed1),4);  // 左前数字
    // OLED_ShowChar(1,9,speed2 >= 0 ? '+' : '-');  // 右后符号
    // OLED_ShowNum(1,10,(int16_t)fabsf(speed2),4);  // 右后数字
    // OLED_ShowChar(2,1,speed3 >= 0 ? '+' : '-');  // 左后符号
    // OLED_ShowNum(2,2,(int16_t)fabsf(speed3),4);  // 左后数字
    // OLED_ShowChar(2,9,speed4 >= 0 ? '+' : '-');  // 右前符号
    // OLED_ShowNum(2,10,(int16_t)fabsf(speed4),4);  // 右前数字

    // 更新上一次的编码器值
    prev_enc1 = enc1;
    prev_enc2 = enc2;
    prev_enc3 = enc3;
    prev_enc4 = enc4;

    // 获取当前偏航角
    float pitch, roll, current_yaw;
    if (MPU6050_DMP_Get_Data(&pitch, &roll, &current_yaw) != 0) {
        // Motor_SetSpeed(id1, 0);
        // Motor_SetSpeed(id2, 0);
        // Motor_SetSpeed(id3, 0);
        // Motor_SetSpeed(id4, 0);
        return;
    }
    *yaw = current_yaw;

    // 计算偏航角误差
    float yaw_error = *target_yaw - *yaw;
    if (yaw_error > 180) yaw_error -= 360;
    else if (yaw_error < -180) yaw_error += 360;

    // 计算速度误差
    float left_speed_error = speed1 - speed3;  // 左侧轮子速度同步
    float right_speed_error = speed2 - speed4;  // 右侧轮子速度同步
    float position_speed_error = (left_speed_error + right_speed_error) / 2;  // 左右两侧速度同步

    // 速度分配 - 修正后的分配方式
    float base_speed = speed;
    
    // 限制PID输出的最大值，防止过度修正
    float max_pid_output = base_speed * 0.3f;
    
    // 计算偏航角PID输出
    float yaw_pid_output = 0.0f;
    if (fabs(yaw_error) > 0.5f) {
        yaw_pid_output = PID_Calculate(&pid_yaw, yaw_error, dt);
        // yaw_pid_output = fmaxf(fminf(yaw_pid_output, max_pid_output*1), -max_pid_output*1);
    } else {
        // 误差小于1度时，重置PID控制器
        PID_Reset(&pid_yaw);
    }
    
    // 计算左右轮组的速度PID输出
    float left_pid_output = PID_Calculate(&pid_front, left_speed_error, dt);
    float right_pid_output = PID_Calculate(&pid_rear, right_speed_error, dt);
    float position_pid_output = PID_Calculate(&pid_position, position_speed_error + yaw_pid_output, dt);
    
    // 限制PID输出
    // left_pid_output = fmaxf(fminf(left_pid_output, max_pid_output*1), -max_pid_output*1);
    // right_pid_output = fmaxf(fminf(right_pid_output, max_pid_output*1), -max_pid_output*1);
    // position_pid_output = fmaxf(fminf(position_pid_output, max_pid_output*1), -max_pid_output*1);
    
    // 左侧轮子 - 正转
    float motor_speed1 = -(base_speed - left_pid_output - position_pid_output - yaw_pid_output);  // 左前
    float motor_speed3 = (base_speed + left_pid_output - position_pid_output - yaw_pid_output);   // 左后
    
    // 右侧轮子 - 反转
    float motor_speed2 = (base_speed - right_pid_output + position_pid_output + yaw_pid_output);  // 右后
    float motor_speed4 = -(base_speed + right_pid_output + position_pid_output + yaw_pid_output); // 右前

    // 限幅
    motor_speed1 = fmaxf(fminf(motor_speed1, 100.0f), -100.0f);
    motor_speed2 = fmaxf(fminf(motor_speed2, 100.0f), -100.0f);
    motor_speed3 = fmaxf(fminf(motor_speed3, 100.0f), -100.0f);
    motor_speed4 = fmaxf(fminf(motor_speed4, 100.0f), -100.0f);

    // 设置电机速度
    Motor_SetSpeed(id1, motor_speed1);
    Motor_SetSpeed(id2, motor_speed2);
    Motor_SetSpeed(id3, motor_speed3);
    Motor_SetSpeed(id4, motor_speed4);
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
        prev_distances[1] = current_distances[1];//把这两个超声波改成右侧的两个
        prev_distances[2] = current_distances[2];
        prev_distances[0] = current_distances[0];//前向超声波
        first_measurement = false;
        return;
    }    
    // 计算移动距离（使用前向超声波传感器测得值的差值）
    float move_distance = (current_distances[0] - prev_distances[0]);
    
    // 如果移动距离太小，不进行角度计算
    if (move_distance < 50.0f) {  // 5cm作为最小移动距离阈值
        return;
    }
    
    for (int i = 1; i < 3; i++) {
        float delta_distance = current_distances[i] - prev_distances[i];
        if (fabsf(delta_distance) > 0.1f) {  // 避免除以接近0的值
            angles[i] = atanf(delta_distance/move_distance) * 180.0f / M_PI;
            valid_angles[valid_count++] = angles[i];
        }
    }
    
    // 更新上一次的距离值
    prev_distances[1] = current_distances[1];
    prev_distances[2] = current_distances[2];
    
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
    
    // 限制速度调整量，保持正负号
    if (speed_adjustment > 0) {
        speed_adjustment = fminf(speed_adjustment, max_adjustment);
    } else {
        speed_adjustment = fmaxf(speed_adjustment, -max_adjustment);
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

#define magnification 0.9
#define magnification_close 0.8

void Adjust_Left_Motors_By_Distance(Motor_ID id1, Motor_ID id3, Motor_ID id2, Motor_ID id4, float distance, float threshold) {
    static int prev_state = 0;  // 0: 未定义, 1: 小于等于31, 2: 大于等于61
    int current_state = 0;
    
    if(distance <= 31) {
        current_state = 1;
        Motor_Rightward(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, 20, &yaw, &target_yaw);
    } else if(distance >= 61) {
        current_state = 2;
        Motor_Rightward(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, -20, &yaw, &target_yaw);
    }
    
    // 只在状态改变时调整target_yaw
    if(current_state != prev_state) {
        if(current_state == 1) {
            target_yaw += 0.3f;
        } else if(current_state == 2) {
            target_yaw -= 0.3f;
        }
        prev_state = current_state;
    }
}

void Adjust_Right_Motors_By_Distance(Motor_ID id2, Motor_ID id4, Motor_ID id1, Motor_ID id3, float distance, float threshold) {
    static int prev_state = 0;  // 0: 未定义, 1: 小于等于31, 2: 大于等于61
    int current_state = 0;
    
    if(distance <= 31) {
        current_state = 1;
        Motor_Rightward(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, -20, &yaw, &target_yaw);
    } else if(distance >= 61) {
        current_state = 2;
        Motor_Rightward(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, 20, &yaw, &target_yaw);
    }
    
    // 只在状态改变时调整target_yaw
    if(current_state != prev_state) {
        if(current_state == 1) {
            target_yaw -= 0.3f;
        } else if(current_state == 2) {
            target_yaw += 0.3f;
        }
        prev_state = current_state;
    }
}

void Adjust_Motors_By_FrontBack_Distance(Motor_ID id1, Motor_ID id4, Motor_ID id2, Motor_ID id3, float distance, float threshold) {
    static int prev_state = 0;  // 0: 未定义, 1: 小于等于34, 2: 大于等于77
    int current_state = 0;
    
    if(distance <= 34) {
        current_state = 1;
        Motor_Straight(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, -12, &yaw, &target_yaw);
    } else if(distance >= 77) {
        current_state = 2;
        Motor_Straight(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, 12, &yaw, &target_yaw);
    }
    
    // 只在状态改变时调整target_yaw
    if(current_state != prev_state) {
        if(current_state == 1) {
            target_yaw -= 0.3f;
        } else if(current_state == 2) {
            target_yaw += 0.3f;
        }
        prev_state = current_state;
    }
}

// void Rotate_90_Degrees(Motor_ID id1, Motor_ID id2, Motor_ID id3, Motor_ID id4, bool clockwise) {
//     static const float MAX_ROTATION_SPEED = 40.0f;  // 最大旋转速度
//     static const float ANGLE_TOLERANCE = 1.0f;      // 角度容差
//     float start_yaw = yaw;  // 记录起始角度
//     float target_angle = start_yaw + (clockwise ? 90.0f : -90.0f);  // 计算目标角度
    
//     // 标准化目标角度到-180到180度范围
//     if (target_angle > 180.0f) {
//         target_angle -= 360.0f;
//     } else if (target_angle < -180.0f) {
//         target_angle += 360.0f;
//     }
    
//     // 设置目标偏航角
//     target_yaw = target_angle;
    
//     // 重置PID控制器
//     PID_Reset(&pid_yaw);
//     PID_Reset(&pid_front);
//     PID_Reset(&pid_rear);
//     PID_Reset(&pid_position);
    
//     // 开始旋转
//     while (1) {
//         // 获取当前偏航角
//         float pitch, roll, current_yaw;
//         if (MPU6050_DMP_Get_Data(&pitch, &roll, &current_yaw) != 0) {
//             continue;  // 如果获取数据失败，继续尝试
//         }
//         yaw = current_yaw;  // 更新全局yaw值

//         // 显示当前角度和目标角度
//         OLED_ShowChar(3,5,yaw >= 0 ? '+' : '-'); 
//         OLED_ShowChar(3,13,target_yaw >= 0 ? '+' : '-'); 
//         OLED_ShowNum(3,14,fabsf(target_yaw),3);
//         OLED_ShowNum(3,6,fabsf(yaw),3);
        
//         // 计算角度误差
//         float angle_error = target_angle - yaw;
//         if (angle_error > 180.0f) angle_error -= 360.0f;
//         else if (angle_error < -180.0f) angle_error += 360.0f;
        
//         // 如果达到目标角度（考虑容差），停止旋转
//         if (fabsf(angle_error) <= ANGLE_TOLERANCE) {
//             // 停止所有电机
//             Motor_SetSpeed(id1, 0);  // 左前
//             Motor_SetSpeed(id2, 0);  // 右后
//             Motor_SetSpeed(id3, 0);  // 左后
//             Motor_SetSpeed(id4, 0);  // 右前
//             break;
//         }
        
//         // 使用PID控制器计算旋转速度
//         float dt = 0.01f;  // 假设10ms的控制周期
//         float pid_output = PID_Calculate(&pid_yaw, angle_error, dt);
        
//         // 限制PID输出范围
//         float rotation_speed = fmaxf(fminf(fabsf(pid_output), MAX_ROTATION_SPEED), 0.0f);
        
//         // 根据旋转方向设置电机速度
//         if (clockwise) {
//             // 顺时针旋转：
//             // 左前(id1)和右后(id2)正转
//             // 左后(id3)和右前(id4)反转
//             Motor_SetSpeed(id1, rotation_speed);   // 左前
//             Motor_SetSpeed(id2, rotation_speed);   // 右后
//             Motor_SetSpeed(id3, -rotation_speed);  // 左后
//             Motor_SetSpeed(id4, -rotation_speed);  // 右前
//         } else {
//             // 逆时针旋转：
//             // 左前(id1)和右后(id2)反转
//             // 左后(id3)和右前(id4)正转
//             Motor_SetSpeed(id1, -rotation_speed);  // 左前
//             Motor_SetSpeed(id2, -rotation_speed);  // 右后
//             Motor_SetSpeed(id3, rotation_speed);   // 左后
//             Motor_SetSpeed(id4, rotation_speed);   // 右前
//         }
        
//         // 短暂延时，避免过于频繁的更新
//         HAL_Delay(10);
//     }
// }

