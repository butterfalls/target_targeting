#include "motor.h"
#include "usart.h"
#include "mpu6050.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

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

void Motor_Forward(Motor_ID id, Motor_ID id2, int16_t speed){
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - prev_time) / 1000.0f;
    prev_time = current_time;

    int32_t enc1 = Motor_GetEncoder(id);
    int32_t enc2 = -Motor_GetEncoder(id2);

    int32_t position_error = enc1 - enc2;

    float pid_output = PID_Calculate(&pid_encoder, position_error, dt);

    float base_speed = speed;

    float speed1 = base_speed - pid_output;
    float speed2 = -(base_speed + pid_output);
    
    speed1 = fmaxf(fminf(speed1, 100.0f), -100.0f);
    speed2 = fmaxf(fminf(speed2, 100.0f), -100.0f);

    Motor_SetSpeed(id, speed1);
    Motor_SetSpeed(id2, speed2);

    Debug_Output("Forward", position_error, pid_output, speed1, speed2);
}

void Motor_Rightward(Motor_ID id, Motor_ID id2, int16_t speed) {
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - prev_time) / 1000.0f;
    prev_time = current_time;

    int32_t enc1 = Motor_GetEncoder(id);
    int32_t enc2 = -Motor_GetEncoder(id2);

    int32_t position_error = enc1 - enc2;

    float pid_output = PID_Calculate(&pid_encoder, position_error, dt);

    float base_speed = speed;

    float speed1 = base_speed - pid_output;
    float speed2 = -(base_speed + pid_output);
    
    speed1 = fmaxf(fminf(speed1, 100.0f), -100.0f);
    speed2 = fmaxf(fminf(speed2, 100.0f), -100.0f);

    Motor_SetSpeed(id, speed1);
    Motor_SetSpeed(id2, speed2);
    
    Debug_Output("RIGHT", position_error, pid_output, speed1, speed2);
}

void Motor_Straight(Motor_ID id1, Motor_ID id2, Motor_ID id3, Motor_ID id4, int16_t speed) {
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - prev_time) / 1000.0f;
    prev_time = current_time;

    // 获取编码器值
    int32_t enc1 = Motor_GetEncoder(id1);
    int32_t enc2 = -Motor_GetEncoder(id2);
    int32_t enc3 = Motor_GetEncoder(id3);
    int32_t enc4 = -Motor_GetEncoder(id4);

    // 计算编码器误差 - 修正后的计算方式
    int32_t left_error = enc1 - enc3;  // 左侧轮子同步
    int32_t right_error = enc2 - enc4;  // 右侧轮子同步
    int32_t position_error = (left_error + right_error) / 2;  // 左右两侧同步

    // 获取当前偏航角
    float pitch, roll, yaw;
    MPU6050_DMP_Get_Data(&pitch, &roll, &yaw);

    // 计算偏航角误差
    float yaw_error = target_yaw - yaw;
    if (yaw_error > 180) yaw_error -= 360;
    else if (yaw_error < -180) yaw_error += 360;

    // PID计算
    float encoder_pid_output = PID_Calculate(&pid_encoder, position_error, dt);
    float yaw_pid_output = PID_Calculate(&pid_yaw, yaw_error, dt);

    // 速度分配 - 修正后的分配方式
    float base_speed = speed;
    // 左侧轮子
    float speed1 = base_speed - encoder_pid_output - yaw_pid_output;  // 左前
    float speed3 = base_speed - encoder_pid_output - yaw_pid_output;  // 左后
    // 右侧轮子
    float speed2 = base_speed + encoder_pid_output + yaw_pid_output;  // 右前
    float speed4 = base_speed + encoder_pid_output + yaw_pid_output;  // 右后

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

    Debug_Output_Yaw("STRAIGHT", yaw_error, yaw_pid_output, speed1, speed2, speed3, speed4);
}

void Motor_TurnLeft90(Motor_ID id1, Motor_ID id2, Motor_ID id3, Motor_ID id4, int16_t speed) {
    static float start_yaw = 0;
    static bool initialized = false;
    static bool turn_completed = false;
    
    if (!initialized) {
        float pitch, roll, yaw;
        MPU6050_DMP_Get_Data(&pitch, &roll, &yaw);
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
        char buf[128];
        sprintf(buf, "%s | YawErr: %5.1f | PID: %6.2f | M1: %5.1f | M2: %5.1f | M3: %5.1f | M4: %5.1f\r\n",
                movement, yaw_error, pid_out, speed1, speed2, speed3, speed4);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
        last_debug = HAL_GetTick();
    }
}
