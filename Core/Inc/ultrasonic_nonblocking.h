#ifndef ULTRASONIC_NONBLOCKING_H
#define ULTRASONIC_NONBLOCKING_H

#include "stm32f4xx_hal.h"

#define MAX_ULTRASONIC_SENSORS 5  // 最大支持的超声波传感器数量

typedef enum {
    US_STATE_IDLE,          // 空闲状态
    US_STATE_TRIG_START,    // 触发开始
    US_STATE_TRIG_PULSE,    // 触发脉冲
    US_STATE_WAIT_ECHO      // 等待回波
} UltrasonicState;

typedef struct {
    // 硬件接口
    GPIO_TypeDef* trig_port;
    uint16_t trig_pin;
    GPIO_TypeDef* echo_port;
    uint16_t echo_pin;
    
    // 状态变量
    UltrasonicState state;
    uint32_t timestamp;     // 基于SysTick的时间戳
    uint32_t pulse_start;   // 回波开始时间
    
    // 测量结果
    float distance;
    uint8_t data_ready;
} UltrasonicSensor;

// 全局变量
extern uint8_t sensor_count;  // 当前活动的传感器数量

// 初始化传感器
void Ultrasonic_Init(UltrasonicSensor* sensor,
                    GPIO_TypeDef* trig_port, uint16_t trig_pin,
                    GPIO_TypeDef* echo_port, uint16_t echo_pin);

// 开始一次新的测量
void Ultrasonic_StartMeasurement(UltrasonicSensor* sensor);

// 状态机更新，需要定期调用
void Ultrasonic_Update(UltrasonicSensor* sensor);

// 获取最新测量结果
float Ultrasonic_GetDistance(UltrasonicSensor* sensor);

// 测试Trig引脚
void Ultrasonic_TestTrig(UltrasonicSensor* sensor);

#endif 