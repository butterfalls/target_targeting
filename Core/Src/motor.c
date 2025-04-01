#include "motor.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/

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

float PID_Calculate(PIDController* pid, float error, float dt) {
    float proportional = pid->Kp * error;

    pid->integral += error * dt;
    pid->integral = fmaxf(fminf(pid->integral, pid->max_integral), -pid->max_integral);

    float derivative = pid->Kd * (error - pid->prev_error) / dt;

    float output = proportional + (pid->Ki * pid->integral) + derivative;

    pid->prev_error = error;

    return output;
}

void Motor_Forward(Motor_ID id, Motor_ID id2, int16_t speed){
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
