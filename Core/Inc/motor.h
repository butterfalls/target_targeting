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
extern float yaw;
extern uint32_t prev_time;
extern TIM_HandleTypeDef htim6;  // 添加 TIM6 句柄

/* Exported functions prototypes ---------------------------------------------*/
void Motor_Init(Motor_ID id,
                TIM_HandleTypeDef* pwm_tim, uint32_t pwm_ch,
                GPIO_TypeDef* in1_port, uint16_t in1_pin,
                GPIO_TypeDef* in2_port, uint16_t in2_pin,
                TIM_HandleTypeDef* encoder_tim);

void Motor_SetSpeed(Motor_ID id, int16_t speed);
int32_t Motor_GetEncoder(Motor_ID id);
void Motor_Rightward(Motor_ID id1, Motor_ID id2, Motor_ID id3, Motor_ID id4, int16_t speed, float* yaw, float* target_yaw);
void Motor_Straight(Motor_ID id1, Motor_ID id2, Motor_ID id3, Motor_ID id4, int16_t speed, float* yaw, float* target_yaw);
void straight_us100(float distance, float* yaw, float* target_yaw);
void Update_Target_Yaw(float* yaw, float* target_yaw);
void Adjust_Speed_By_Side_Distance(Motor_ID id1, Motor_ID id2, int16_t base_speed, float side_distance, float target_distance);

// 添加时间相关函数
uint32_t Get_Time_Difference(void);  // 获取时间差（秒）
void Reset_Timer(void);  // 重置计时器

// 新增的电机调整函数
void Adjust_Left_Motors_By_Distance(Motor_ID id1, Motor_ID id3, Motor_ID id2, Motor_ID id4, float distance, float threshold);
void Adjust_Right_Motors_By_Distance(Motor_ID id2, Motor_ID id4, Motor_ID id1, Motor_ID id3, float distance, float threshold);
void Adjust_Motors_By_FrontBack_Distance(Motor_ID id1, Motor_ID id4, Motor_ID id2, Motor_ID id3, float distance, float threshold);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
