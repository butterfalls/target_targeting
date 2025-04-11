#include "ultrasonic_nonblocking.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

// 系统时钟频率（Hz）
#define SYSCLK_FREQ 168000000

// 触发脉冲宽度（μs）
#define TRIG_PULSE_WIDTH 20

// 距离滤波参数
#define MIN_VALID_DISTANCE 2.0f  // 最小有效距离（厘米）
#define MAX_VALID_DISTANCE 400.0f  // 最大有效距离（厘米）
#define FILTER_SAMPLES 5  // 滤波样本数

// 全局传感器数组
static UltrasonicSensor* active_sensors[MAX_ULTRASONIC_SENSORS] = {0};
uint8_t sensor_count = 0;  // 当前活动的传感器数量

// 距离滤波缓冲区
static float distance_buffer[MAX_ULTRASONIC_SENSORS][FILTER_SAMPLES] = {0};
static uint8_t buffer_index[MAX_ULTRASONIC_SENSORS] = {0};

// 获取微秒级时间戳
static uint32_t get_us_timestamp(void) {
    // 使用SysTick计数器获取微秒级时间戳
    uint32_t ticks = SysTick->VAL;
    uint32_t ticks_per_us = SYSCLK_FREQ / 1000000;
    uint32_t us = (ticks / ticks_per_us) + (HAL_GetTick() * 1000);
    return us;
}

void Ultrasonic_Init(UltrasonicSensor* sensor,
                    GPIO_TypeDef* trig_port, uint16_t trig_pin,
                    GPIO_TypeDef* echo_port, uint16_t echo_pin) {
    if (sensor_count >= MAX_ULTRASONIC_SENSORS) return;
    
    // 保存GPIO信息
    sensor->trig_port = trig_port;
    sensor->trig_pin = trig_pin;
    sensor->echo_port = echo_port;
    sensor->echo_pin = echo_pin;
    
    // 配置Trig为输出
    GPIO_InitTypeDef gpio_init = {
        .Pin = trig_pin,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH
    };
    HAL_GPIO_Init(trig_port, &gpio_init);
    
    // 确保Trig初始状态为低电平
    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);
    
    // 配置Echo为中断输入
    gpio_init.Pin = echo_pin;
    gpio_init.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio_init.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(echo_port, &gpio_init);
    
    sensor->state = US_STATE_IDLE;
    sensor->data_ready = 0;
    sensor->distance = 0.0f;
    
    // 初始化滤波缓冲区
    for (int i = 0; i < FILTER_SAMPLES; i++) {
        distance_buffer[sensor_count][i] = 0.0f;
    }
    buffer_index[sensor_count] = 0;
    
    // 添加到活动传感器数组
    active_sensors[sensor_count++] = sensor;
    
    // 调试输出
    char debug_msg[50];
    sprintf(debug_msg, "ultrasonic_init: Trig=%d, Echo=%d\r\n", trig_pin, echo_pin);
    HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), 100);
}

void Ultrasonic_StartMeasurement(UltrasonicSensor* sensor) {
    if (sensor->state == US_STATE_IDLE) {
        sensor->state = US_STATE_TRIG_START;
        sensor->timestamp = HAL_GetTick();
        
        // 调试输出已注释掉
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
            
            // 调试输出已注释掉
            break;
            
        case US_STATE_TRIG_PULSE:
            // 触发脉冲维持阶段
            if ((now - sensor->timestamp) >= 2) { // 等待2ms低电平
                HAL_GPIO_WritePin(sensor->trig_port, sensor->trig_pin, GPIO_PIN_SET);
                sensor->timestamp = now;
                sensor->state = US_STATE_WAIT_ECHO;
                
                // 调试输出已注释掉
            }
            break;
            
        case US_STATE_WAIT_ECHO:
            // 维持触发脉冲
            if ((now - sensor->timestamp) >= 1) { // 等待1ms高电平
                HAL_GPIO_WritePin(sensor->trig_port, sensor->trig_pin, GPIO_PIN_RESET);
                sensor->state = US_STATE_IDLE;
                
                // 调试输出已注释掉
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
                // 上升沿，记录开始时间（微秒级）
                s->pulse_start = get_us_timestamp();
            } else {
                // 下降沿，计算时间（微秒级）
                uint32_t end_time = get_us_timestamp();
                uint32_t duration_us = end_time - s->pulse_start;
                
                // 计算距离（厘米）：声速340m/s = 0.034cm/μs
                float raw_distance = (duration_us * 0.034f) / 2.0f;
                
                // 检查距离是否在有效范围内
                if (raw_distance >= MIN_VALID_DISTANCE && raw_distance <= MAX_VALID_DISTANCE) {
                    // 更新滤波缓冲区
                    distance_buffer[i][buffer_index[i]] = raw_distance;
                    buffer_index[i] = (buffer_index[i] + 1) % FILTER_SAMPLES;
                    
                    // 计算中值滤波
                    float temp_buffer[FILTER_SAMPLES];
                    for (int j = 0; j < FILTER_SAMPLES; j++) {
                        temp_buffer[j] = distance_buffer[i][j];
                    }
                    
                    // 简单冒泡排序
                    for (int j = 0; j < FILTER_SAMPLES - 1; j++) {
                        for (int k = 0; k < FILTER_SAMPLES - j - 1; k++) {
                            if (temp_buffer[k] > temp_buffer[k + 1]) {
                                float temp = temp_buffer[k];
                                temp_buffer[k] = temp_buffer[k + 1];
                                temp_buffer[k + 1] = temp;
                            }
                        }
                    }
                    
                    // 取中值
                    s->distance = temp_buffer[FILTER_SAMPLES / 2];
                } else {
                    // 距离无效，保持上一次的有效值
                    // 如果所有值都无效，则设为0
                    if (s->distance < MIN_VALID_DISTANCE || s->distance > MAX_VALID_DISTANCE) {
                        s->distance = 0.0f;
                    }
                }
                
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

// 测试Trig引脚
void Ultrasonic_TestTrig(UltrasonicSensor* sensor) {
    // 输出测试信息
    char debug_msg[50];
    sprintf(debug_msg, "Testing Trig pin: %d\r\n", (int)sensor->trig_pin);
    HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), 100);
    
    // 设置Trig为高电平
    HAL_GPIO_WritePin(sensor->trig_port, sensor->trig_pin, GPIO_PIN_SET);
    HAL_Delay(100);  // 等待100ms
    
    // 设置Trig为低电平
    HAL_GPIO_WritePin(sensor->trig_port, sensor->trig_pin, GPIO_PIN_RESET);
    HAL_Delay(100);  // 等待100ms
    
    // 再次设置Trig为高电平
    HAL_GPIO_WritePin(sensor->trig_port, sensor->trig_pin, GPIO_PIN_SET);
    HAL_Delay(100);  // 等待100ms
    
    // 再次设置Trig为低电平
    HAL_GPIO_WritePin(sensor->trig_port, sensor->trig_pin, GPIO_PIN_RESET);
    
    // 输出测试完成信息
    sprintf(debug_msg, "Trig pin test completed\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), 100);
}