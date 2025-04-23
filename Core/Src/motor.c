#include "motor.h"
#include "usart.h"
#include "mpu6050.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "OLED.h"

// 定义圆周率
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Private variables ---------------------------------------------------------*/
float target_yaw = 0.0f;

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

void Motor_Forward(Motor_ID id, Motor_ID id2, int16_t speed){
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - prev_time) / 1000.0f;
    prev_time = current_time;

    // 获取编码器值
    int32_t enc1 = Motor_GetEncoder(id);
    int32_t enc2 = -Motor_GetEncoder(id2);

    // 获取当前偏航角
    float pitch, roll, yaw;
    MPU6050_DMP_Get_Data(&pitch, &roll, &yaw);

    // 计算偏航角误差
    float yaw_error = target_yaw - yaw;
    if (yaw_error > 180) yaw_error -= 360;
    else if (yaw_error < -180) yaw_error += 360;

    // 先计算偏航角PID输出
    float yaw_pid_output = PID_Calculate(&pid_yaw, yaw_error, dt);

    // 计算编码器误差
    int32_t position_error = enc1 - enc2;

    // 使用偏航角PID输出作为编码器PID的目标值
    float encoder_pid_output = PID_Calculate(&pid_encoder, position_error - yaw_pid_output, dt);

    float base_speed = speed;

    float speed1 = base_speed - encoder_pid_output - yaw_pid_output;
    float speed2 = -(base_speed + encoder_pid_output + yaw_pid_output);
    
    speed1 = fmaxf(fminf(speed1, 100.0f), -100.0f);
    speed2 = fmaxf(fminf(speed2, 100.0f), -100.0f);

    Motor_SetSpeed(id, speed1);
    Motor_SetSpeed(id2, speed2);

    Debug_Output("Forward", position_error, encoder_pid_output, speed1, speed2);
}

// 基于超声波数据计算垄的平行度
float Calculate_Furrow_Parallel(float distance1, float distance2, float* yaw_target, bool* use_ultrasonic_control) {
    // 默认不使用超声波控制
    *use_ultrasonic_control = false;
    
    // 检查数据有效性
    if (distance1 <= 0 || distance2 <= 0) {
        return 0.0f;
    }
    
    float distance_diff = distance1 - distance2;
    
    const float SENSOR_SPACING = 100.0f;  // 需要根据实际安装距离调整
    
    float angle_rad = atan2(distance_diff, SENSOR_SPACING);
    
    float angle_deg = angle_rad * 180.0f / M_PI;
    
    const float ANGLE_THRESHOLD = 1.0f;
    
    if (fabs(angle_deg) > ANGLE_THRESHOLD) {
        *use_ultrasonic_control = true;
        
        *yaw_target = angle_deg;

        *yaw_target = fmaxf(fminf(*yaw_target, 15.0f), -15.0f);
    }
    
    return angle_deg;
}

void Motor_Rightward(Motor_ID id1, Motor_ID id2, Motor_ID id3, Motor_ID id4, int16_t speed) {
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
    float pitch, roll, yaw;
    if (MPU6050_DMP_Get_Data(&pitch, &roll, &yaw) != 0) {
        Motor_SetSpeed(id1, 0);
        Motor_SetSpeed(id2, 0);
        Motor_SetSpeed(id3, 0);
        Motor_SetSpeed(id4, 0);
        return;
    }
    OLED_ShowNum(3,5,yaw,3);

    // 获取超声波数据
    float distances[4];
    US100_GetAllValidDistances(distances);
    
    // 计算垄的平行度
    float ultrasonic_yaw_target;
    bool use_ultrasonic_control;
    float parallel_ratio = Calculate_Furrow_Parallel(distances[1], distances[3], &ultrasonic_yaw_target, &use_ultrasonic_control);
    /*如果不再使用超声波调整平行度，打开注释内容*/
    // use_ultrasonic_control = false;
    float yaw_error;
    if (use_ultrasonic_control) {
        // 使用超声波计算的偏航角目标值
        yaw_error = target_yaw - yaw + parallel_ratio;
    } else {
        // 使用预设的目标偏航角
        yaw_error = target_yaw - yaw;
        if (yaw_error > 180) yaw_error -= 360;
        else if (yaw_error < -180) yaw_error += 360;
    }

    // 计算编码器误差 - 修正后的计算方式
    int32_t front_error = enc1 - enc2;  // 前侧轮子同步
    int32_t rear_error = enc3 - enc4;   // 后侧轮子同步
    int32_t position_error = (front_error + rear_error) / 2;  // 前后两侧同步

    // 速度分配 - 修正后的分配方式
    float base_speed = speed;
    
    // 限制PID输出的最大值，防止过度修正
    float max_pid_output = base_speed * 0.3f;  // 降低PID输出最大值为基准速度的30%
    
    float yaw_pid_output = 0.0f;
    float encoder_pid_output = 0.0f;
    
    // 只有当偏航角误差大于1度时才进行PID调整
    if (fabs(yaw_error) > 1.0f) {
        yaw_pid_output = PID_Calculate(&pid_yaw, yaw_error, dt);
        yaw_pid_output = fmaxf(fminf(yaw_pid_output, max_pid_output), -max_pid_output);
    } else {
        // 误差小于1度时，重置PID控制器
        PID_Reset(&pid_yaw);
    }
    
    // 使用偏航角PID输出作为编码器PID的目标值
    encoder_pid_output = PID_Calculate(&pid_encoder, position_error - yaw_pid_output, dt);
    encoder_pid_output = fmaxf(fminf(encoder_pid_output, max_pid_output), -max_pid_output);
    
    // 前侧轮子 - 向内运动
    float speed1 = (base_speed - encoder_pid_output - yaw_pid_output);  // 左前
    float speed2 = (base_speed + encoder_pid_output + yaw_pid_output);  // 右前
    
    // 后侧轮子 - 向内运动
    float speed3 = (base_speed - encoder_pid_output - yaw_pid_output);  // 左后
    float speed4 = (base_speed + encoder_pid_output + yaw_pid_output);  // 右后

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

    // 输出调试信息，包括编码器误差和PID输出
    // Debug_Output_Yaw("RIGHTWARD", yaw_error, yaw_pid_output, speed1, speed2, speed3, speed4);
}

void Motor_Straight(Motor_ID id1, Motor_ID id2, Motor_ID id3, Motor_ID id4, int16_t speed) {
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
    float pitch, roll, yaw;
    if (MPU6050_DMP_Get_Data(&pitch, &roll, &yaw) != 0) {
        Motor_SetSpeed(id1, 0);
        Motor_SetSpeed(id2, 0);
        Motor_SetSpeed(id3, 0);
        Motor_SetSpeed(id4, 0);
        return;
    }
    OLED_ShowNum(3,5,yaw,3);

    // 计算偏航角误差
    float yaw_error = target_yaw - yaw;
    if (yaw_error > 180) yaw_error -= 360;
    else if (yaw_error < -180) yaw_error += 360;

    // 计算编码器误差 - 修正后的计算方式
    int32_t left_error = enc1 - enc3;  // 左侧轮子同步
    int32_t right_error = enc2 - enc4;  // 右侧轮子同步
    int32_t position_error = (left_error + right_error) / 2;  // 左右两侧同步

    // 速度分配 - 修正后的分配方式
    float base_speed = speed;
    
    // 限制PID输出的最大值，防止过度修正
    float max_pid_output = base_speed * 0.3f;  // 降低PID输出最大值为基准速度的30%
    
    float yaw_pid_output = 0.0f;
    float encoder_pid_output = 0.0f;
    
    // 只有当偏航角误差大于1度时才进行PID调整
    if (fabs(yaw_error) > 1.0f) {
        yaw_pid_output = PID_Calculate(&pid_yaw, yaw_error, dt);
        yaw_pid_output = fmaxf(fminf(yaw_pid_output, max_pid_output), -max_pid_output);
    } else {
        // 误差小于1度时，重置PID控制器
        PID_Reset(&pid_yaw);
    }
    
    // 使用偏航角PID输出作为编码器PID的目标值
    encoder_pid_output = PID_Calculate(&pid_encoder, position_error - yaw_pid_output, dt);
    encoder_pid_output = fmaxf(fminf(encoder_pid_output, max_pid_output), -max_pid_output);
    
    // 左侧轮子 - 正转
    float speed1 = -(base_speed - encoder_pid_output - yaw_pid_output);  
    float speed3 = (base_speed - encoder_pid_output - yaw_pid_output);  
    
    // 右侧轮子 - 反转
    float speed2 = (base_speed + encoder_pid_output + yaw_pid_output);  
    float speed4 = -(base_speed + encoder_pid_output + yaw_pid_output);  // 右后

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

    // 输出调试信息，包括编码器误差和PID输出
    Debug_Output_Yaw("STRAIGHT", yaw_error, yaw_pid_output, speed1, speed2, speed3, speed4);
}

void Motor_TurnLeft90(Motor_ID id1, Motor_ID id2, Motor_ID id3, Motor_ID id4, int16_t speed) {
    static float start_yaw = 0;
    static bool initialized = false;
    static bool turn_completed = false;
    
    if (!initialized) {
        float pitch, roll, yaw;
        MPU6050_DMP_Get_Data(&pitch, &roll, &yaw);
        OLED_ShowNum(3,5,yaw,3);
        start_yaw = yaw;
        target_yaw = start_yaw - 90.0f;
        if (target_yaw < -180.0f) target_yaw += 360.0f;
        initialized = true;
        turn_completed = false;
    }

    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - prev_time) / 1000.0f;
    prev_time = current_time;

    // 获取当前偏航角
    float pitch, roll, yaw;
    MPU6050_DMP_Get_Data(&pitch, &roll, &yaw);
    OLED_ShowNum(3,5,yaw,3);

    // 计算偏航角误差
    float yaw_error = target_yaw - yaw;
    if (yaw_error > 180) yaw_error -= 360;
    else if (yaw_error < -180) yaw_error += 360;

    // 检查是否完成转向
    if (!turn_completed && fabs(yaw_error) < 1.0f) {
        turn_completed = true;
        // 更新全局目标偏航角，使其与当前偏航角一致
        // 这样在转向后衔接直行时，直行函数会使用正确的目标偏航角
        target_yaw = yaw;
    }

    // PID计算
    float yaw_pid_output = PID_Calculate(&pid_yaw, yaw_error, dt);

    // 速度分配 - 所有电机同向转动以实现原地左转
    float turn_speed = yaw_pid_output;
    float speed1 = turn_speed;
    float speed2 = turn_speed;
    float speed3 = -turn_speed;
    float speed4 = -turn_speed;

    // 限幅
    speed1 = fmaxf(fminf(speed1, speed), -speed);
    speed2 = fmaxf(fminf(speed2, speed), -speed);
    speed3 = fmaxf(fminf(speed3, speed), -speed);
    speed4 = fmaxf(fminf(speed4, speed), -speed);

    // 设置电机速度
    Motor_SetSpeed(id1, speed1);
    Motor_SetSpeed(id2, speed2);
    Motor_SetSpeed(id3, speed3);
    Motor_SetSpeed(id4, speed4);

    Debug_Output_Yaw("TURN_LEFT", yaw_error, yaw_pid_output, speed1, speed2, speed3, speed4);

    // 如果转向完成，重置初始化标志
    if (turn_completed) {
        initialized = false;
    }
}

void Motor_TurnLeft90_Blocking(Motor_ID id1, Motor_ID id2, Motor_ID id3, Motor_ID id4, int16_t speed) {
    float pitch, roll, yaw;
    MPU6050_DMP_Get_Data(&pitch, &roll, &yaw);
    OLED_ShowNum(3,5,yaw,3);
    float start_yaw = yaw;
    float target_yaw_local = start_yaw - 90.0f;
    if (target_yaw_local < -180.0f) target_yaw_local += 360.0f;
    
    bool turn_completed = false;
    
    while (!turn_completed) {
        uint32_t current_time = HAL_GetTick();
        float dt = (current_time - prev_time) / 1000.0f;
        prev_time = current_time;
        
        // 获取当前偏航角
        MPU6050_DMP_Get_Data(&pitch, &roll, &yaw);
        OLED_ShowNum(3,5,yaw,3);
        
        // 计算偏航角误差
        float yaw_error = target_yaw_local - yaw;
        if (yaw_error > 180) yaw_error -= 360;
        else if (yaw_error < -180) yaw_error += 360;
        
        // 检查是否完成转向
        if (fabs(yaw_error) < 1.0f) {
            turn_completed = true;
            // 更新全局目标偏航角
            target_yaw = yaw;
            // 停止所有电机
            Motor_SetSpeed(id1, 0);
            Motor_SetSpeed(id2, 0);
            Motor_SetSpeed(id3, 0);
            Motor_SetSpeed(id4, 0);
            
            char buf[64];
            sprintf(buf, "Turn completed. Current yaw: %.2f\r\n", yaw);
            HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
            break;
        }
        
        // PID计算
        float yaw_pid_output = PID_Calculate(&pid_yaw, yaw_error, dt);
        
        // 速度分配 - 所有电机同向转动以实现原地左转
        float turn_speed = yaw_pid_output;
        float speed1 = turn_speed;
        float speed2 = turn_speed;
        float speed3 = -turn_speed;
        float speed4 = -turn_speed;
        
        // 限幅
        speed1 = fmaxf(fminf(speed1, speed), -speed);
        speed2 = fmaxf(fminf(speed2, speed), -speed);
        speed3 = fmaxf(fminf(speed3, speed), -speed);
        speed4 = fmaxf(fminf(speed4, speed), -speed);
        
        // 设置电机速度
        Motor_SetSpeed(id1, speed1);
        Motor_SetSpeed(id2, speed2);
        Motor_SetSpeed(id3, speed3);
        Motor_SetSpeed(id4, speed4);
        
        Debug_Output_Yaw("TURN_LEFT_BLOCKING", yaw_error, yaw_pid_output, speed1, speed2, speed3, speed4);
        
        HAL_Delay(10);  // 短暂延时，避免CPU占用过高
    }
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

void Debug_Output_Yaw(const char* movement, float yaw_error, float pid_out, float speed1, float speed2, float speed3, float speed4) {
    static uint32_t last_debug = 0;
    if (HAL_GetTick() - last_debug > 100) {
        // 获取编码器值
        int32_t enc1 = Motor_GetEncoder(MOTOR_1);
        int32_t enc2 = -Motor_GetEncoder(MOTOR_2);
        int32_t enc3 = Motor_GetEncoder(MOTOR_3);
        int32_t enc4 = -Motor_GetEncoder(MOTOR_4);
        
        char buf[200];
        sprintf(buf, "%s | YawErr: %5.1f | PID: %6.2f | enc1:%5ld, enc2:%5ld, enc3:%5ld, enc4:%5ld | EncPID: %6.2f | M1: %5.1f | M2: %5.1f | M3: %5.1f | M4: %5.1f\r\n",
                movement, yaw_error, pid_out, enc1, enc2, enc3, enc4, pid_encoder.integral, speed1, speed2, speed3, speed4);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
        last_debug = HAL_GetTick();
    }
}

void straight_us100(float distance)/*需要调整参数*/
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
    
    // 使用四轮直行控制
    Motor_Rightward(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, speed);
    
    // 输出调试信息
    char debug_buf[64];
    sprintf(debug_buf, "Distance: %.0f mm, Speed: %d\r\n", distance_forward, speed);
    HAL_UART_Transmit(&huart1, (uint8_t*)debug_buf, strlen(debug_buf), 100);

}