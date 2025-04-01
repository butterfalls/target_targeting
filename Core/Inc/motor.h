#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
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

/* Exported variables -------------------------------------------------------*/
extern Motor motors[MOTOR_COUNT];
extern PIDController pid;  // 前轮方向
extern PIDController pid2; // 左右方向
extern float target_speed;
extern uint32_t prev_time;

/* Exported functions prototypes ---------------------------------------------*/
void Motor_Init(Motor_ID id,
                TIM_HandleTypeDef* pwm_tim, uint32_t pwm_ch,
                GPIO_TypeDef* in1_port, uint16_t in1_pin,
                GPIO_TypeDef* in2_port, uint16_t in2_pin,
                TIM_HandleTypeDef* encoder_tim);

void Motor_SetSpeed(Motor_ID id, int16_t speed);
int32_t Motor_GetEncoder(Motor_ID id);
float PID_Calculate(PIDController* pid, float error, float dt);
void Motor_Forward(Motor_ID id, Motor_ID id2, int16_t speed);
void Motor_Rightward(Motor_ID id, Motor_ID id2, int16_t speed);
void Debug_Output(const char* movement, int32_t error, float pid_out, float speed1, float speed2);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
