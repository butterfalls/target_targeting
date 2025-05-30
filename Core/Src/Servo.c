#include "servo.h"

// 定时器时钟频率（APB2=84MHz，TIMx使用84MHz）
#define TIMER_CLK_FREQ 168000000  // 单位：Hz

void Servo_Init(Servo* servo, TIM_HandleTypeDef* timer, uint32_t channel,
                GPIO_TypeDef* gpio_port, uint16_t gpio_pin) 
{
    servo->timer = timer;
    servo->channel = channel;
    servo->gpio_port = gpio_port;
    servo->gpio_pin = gpio_pin;
    servo->pulse_width = SERVO_MIN_PULSE;

    // 配置PWM周期
    uint32_t period_cycles = (TIMER_CLK_FREQ / 1000000) * SERVO_PWM_PERIOD / 
                            (timer->Init.Prescaler + 1);
    __HAL_TIM_SET_AUTORELOAD(timer, period_cycles - 1);

    // 启动PWM
    HAL_TIM_PWM_Start(timer, channel);
    Servo_SetPulse(servo, servo->pulse_width);
}

void Servo_SetAngle(Servo* servo, float angle) {
    // 角度限幅
    angle = (angle < 0) ? 0 : (angle > 180) ? 180 : angle;
    
    // 计算脉宽
    uint32_t pulse = SERVO_MIN_PULSE + 
                    (uint32_t)((SERVO_MAX_PULSE - SERVO_MIN_PULSE) * angle / 180.0f);
    
    Servo_SetPulse(servo, pulse);
}

void Servo_SetPulse(Servo* servo, uint32_t pulse_us) {
    // 计算比较寄存器值
    uint32_t pulse_cycles = (TIMER_CLK_FREQ / 1000000) * pulse_us / 
                           (servo->timer->Init.Prescaler + 1);
    
    // 设置比较值
    switch(servo->channel) {
        case TIM_CHANNEL_1:
            __HAL_TIM_SET_COMPARE(servo->timer, TIM_CHANNEL_1, pulse_cycles);
            break;
        case TIM_CHANNEL_2:
            __HAL_TIM_SET_COMPARE(servo->timer, TIM_CHANNEL_2, pulse_cycles);
            break;
        // 添加其他通道...
    }
    servo->pulse_width = pulse_us;
}

// 添加缓动函数
void Servo_SmoothMove(Servo* servo, float target_angle, uint32_t duration_ms) {
    float start_angle = (servo->pulse_width - SERVO_MIN_PULSE) * 180.0f / 
                       (SERVO_MAX_PULSE - SERVO_MIN_PULSE);
    uint32_t steps = duration_ms / 20; // 每20ms更新一次
    
    for(uint32_t i=0; i<=steps; i++) {
        float angle = start_angle + (target_angle - start_angle) * 
                     (1 - cosf(i * 3.14159265358979323846 / steps)) / 2;
        Servo_SetAngle(servo, angle);
        HAL_Delay(20);
    }
}
