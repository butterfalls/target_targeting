#include "ultrasonic_nonblocking.h"

// SysTick时间基准为1ms分辨率
#define SYSTICK_FREQ 1000

// 触发脉冲宽度（μs）
#define TRIG_PULSE_WIDTH 20

// 全局传感器数组
static UltrasonicSensor* active_sensors[MAX_ULTRASONIC_SENSORS] = {0};
static uint8_t sensor_count = 0;

void Ultrasonic_Init(UltrasonicSensor* sensor) {
    if (sensor_count >= MAX_ULTRASONIC_SENSORS) return;
    
    // 配置Trig为输出
    GPIO_InitTypeDef gpio_init = {
        .Pin = sensor->trig_pin,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH
    };
    HAL_GPIO_Init(sensor->trig_port, &gpio_init);
    
    // 配置Echo为中断输入
    gpio_init.Pin = sensor->echo_pin;
    gpio_init.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio_init.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(sensor->echo_port, &gpio_init);
    
    sensor->state = US_STATE_IDLE;
    sensor->data_ready = 0;
    
    // 添加到活动传感器数组
    active_sensors[sensor_count++] = sensor;
}

void Ultrasonic_StartMeasurement(UltrasonicSensor* sensor) {
    if (sensor->state == US_STATE_IDLE) {
        sensor->state = US_STATE_TRIG_START;
        sensor->timestamp = HAL_GetTick();
    }
}

void Ultrasonic_Update(UltrasonicSensor* sensor) {
    uint32_t now = HAL_GetTick();
    
    switch (sensor->state) {
        case US_STATE_TRIG_START:
            // 触发开始，设置Trig为低电平
            HAL_GPIO_WritePin(sensor->trig_port, sensor->trig_pin, GPIO_PIN_RESET);
            sensor->timestamp = now;
            sensor->state = US_STATE_TRIG_PULSE;
            break;
            
        case US_STATE_TRIG_PULSE:
            // 触发脉冲维持阶段
            if ((now - sensor->timestamp) >= 2) { // 等待2ms低电平
                HAL_GPIO_WritePin(sensor->trig_port, sensor->trig_pin, GPIO_PIN_SET);
                sensor->timestamp = now;
                sensor->state = US_STATE_WAIT_ECHO;
            }
            break;
            
        case US_STATE_WAIT_ECHO:
            // 维持触发脉冲
            if ((now - sensor->timestamp) * 1000 >= TRIG_PULSE_WIDTH) { // 转换为μs
                HAL_GPIO_WritePin(sensor->trig_port, sensor->trig_pin, GPIO_PIN_RESET);
                sensor->state = US_STATE_IDLE;
            }
            break;
            
        default:
            break;
    }
}

// Echo中断回调
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    for (uint8_t i = 0; i < sensor_count; i++) {
        UltrasonicSensor* s = active_sensors[i];
        
        if (GPIO_Pin == s->echo_pin) {
            if (HAL_GPIO_ReadPin(s->echo_port, s->echo_pin)) {
                // 上升沿，记录开始时间
                s->pulse_start = HAL_GetTick();
            } else {
                // 下降沿，计算时间
                uint32_t duration = HAL_GetTick() - s->pulse_start;
                s->distance = (duration * 34300.0f) / (2 * SYSTICK_FREQ); // 单位：厘米
                s->data_ready = 1;
            }
        }
    }
}

float Ultrasonic_GetDistance(UltrasonicSensor* sensor) {
    if (sensor->data_ready) {
        sensor->data_ready = 0;
        return sensor->distance;
    }
    return -1.0f; // 无效数据
}