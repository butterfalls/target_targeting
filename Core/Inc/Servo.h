#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f4xx_hal.h"

// 舵机参数配置（根据实际型号调整）
#define SERVO_PWM_PERIOD    20000   // PWM周期20ms（单位：微秒）
#define SERVO_MIN_PULSE     500     // 0度对应脉宽（微秒）
#define SERVO_MAX_PULSE     2500    // 180度对应脉宽（微秒）

typedef struct {
    TIM_HandleTypeDef* timer;  // 使用的定时器
    uint32_t channel;          // 定时器通道
    GPIO_TypeDef* gpio_port;   // 舵机信号线GPIO端口
    uint16_t gpio_pin;         // 舵机信号线GPIO引脚
    uint32_t pulse_width;      // 当前脉宽（微秒）
} Servo;

// 初始化舵机
void Servo_Init(Servo* servo, TIM_HandleTypeDef* timer, uint32_t channel, 
                GPIO_TypeDef* gpio_port, uint16_t gpio_pin);

// 设置舵机角度（0-180度）
void Servo_SetAngle(Servo* servo, float angle);

// 直接设置脉宽（微秒）
void Servo_SetPulse(Servo* servo, uint32_t pulse_us);

// 添加缓动函数
void Servo_SmoothMove(Servo* servo, float target_angle, uint32_t duration_ms);

// 添加函数声明
void Servo_SetMultipleAngles(Servo* servos[], float angles[], uint8_t count, uint32_t duration_ms);

#endif