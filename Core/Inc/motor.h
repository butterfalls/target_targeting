#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "pid_controller.h"
#include <stdbool.h>

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

/* Exported variables -------------------------------------------------------*/
extern Motor motors[MOTOR_COUNT];
extern float target_speed;
extern float target_yaw;   // 目标偏航角
extern uint32_t prev_time;

/* Exported functions prototypes ---------------------------------------------*/
void Motor_Init(Motor_ID id,
                TIM_HandleTypeDef* pwm_tim, uint32_t pwm_ch,
                GPIO_TypeDef* in1_port, uint16_t in1_pin,
                GPIO_TypeDef* in2_port, uint16_t in2_pin,
                TIM_HandleTypeDef* encoder_tim);

void Motor_SetSpeed(Motor_ID id, int16_t speed);
int32_t Motor_GetEncoder(Motor_ID id);
void Motor_Forward(Motor_ID id, Motor_ID id2, int16_t speed);
void Motor_Rightward(Motor_ID id, Motor_ID id2, Motor_ID id3, Motor_ID id4, int16_t speed);
void Motor_Straight(Motor_ID id1, Motor_ID id2, Motor_ID id3, Motor_ID id4, int16_t speed);
void Motor_TurnLeft90(Motor_ID id1, Motor_ID id2, Motor_ID id3, Motor_ID id4, int16_t speed);
void Motor_TurnLeft90_Blocking(Motor_ID id1, Motor_ID id2, Motor_ID id3, Motor_ID id4, int16_t speed);
void Debug_Output(const char* movement, int32_t error, float pid_out, float speed1, float speed2);
void Debug_Output_Yaw(const char* movement, float yaw_error, float pid_out, float speed1, float speed2, float speed3, float speed4);
void straight_us100(float distance);

// 基于超声波数据计算垄的平行度
float Calculate_Furrow_Parallel(float distance1, float distance2, float* yaw_target, bool* use_ultrasonic_control);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
