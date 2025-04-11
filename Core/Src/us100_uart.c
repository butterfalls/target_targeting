#include "us100_uart.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

// 全局传感器数组
static US100Sensor* active_sensors[MAX_US100_SENSORS] = {0};
uint8_t us100_sensor_count = 0;  // 当前活动的传感器数量

// 超时时间（毫秒）
#define US100_TIMEOUT_MS 300  // 增加超时时间到300ms

void US100_Init(US100Sensor* sensor, UART_HandleTypeDef* uart) {
    if (us100_sensor_count >= MAX_US100_SENSORS) return;
    
    // 保存串口句柄
    sensor->uart = uart;
    
    // 初始化状态
    sensor->state = US100_STATE_IDLE;
    sensor->data_ready = 0;
    sensor->distance = 0.0f;
    sensor->rx_index = 0;
    
    // 添加到活动传感器数组
    active_sensors[us100_sensor_count++] = sensor;
    
    // 确保串口已初始化
    if (HAL_UART_GetState(uart) != HAL_UART_STATE_READY) {
        if (HAL_UART_Init(uart) != HAL_OK) {
            return;
        }
    }
    
    // 启动串口接收
    HAL_UART_Receive_IT(uart, &sensor->rx_buffer[0], 1);
}

void US100_StartMeasurement(US100Sensor* sensor) {
    // 如果传感器不在IDLE状态，先重置状态
    if (sensor->state != US100_STATE_IDLE) {
        sensor->state = US100_STATE_IDLE;
        sensor->rx_index = 0;
        HAL_UART_Receive_IT(sensor->uart, &sensor->rx_buffer[0], 1);
    }
    
    // 开始新的测量
    sensor->state = US100_STATE_SENDING;
    sensor->timestamp = HAL_GetTick();
    
    // 发送读取距离命令
    uint8_t cmd = US100_CMD_READ_DISTANCE;
    HAL_UART_Transmit(sensor->uart, &cmd, 1, 100);
}

void US100_Update(US100Sensor* sensor) {
    uint32_t now = HAL_GetTick();
    
    switch (sensor->state) {
        case US100_STATE_SENDING:
            if ((now - sensor->timestamp) >= 10) {
                sensor->state = US100_STATE_WAITING;
                sensor->timestamp = now;
            }
            break;
            
        case US100_STATE_WAITING:
            if ((now - sensor->timestamp) >= US100_TIMEOUT_MS) {
                sensor->state = US100_STATE_IDLE;
                sensor->rx_index = 0;
                HAL_UART_Receive_IT(sensor->uart, &sensor->rx_buffer[0], 1);
            }
            break;
            
        case US100_STATE_RECEIVING:
            if (sensor->rx_index >= 2) {
                uint16_t raw_distance = (sensor->rx_buffer[1] << 8) | sensor->rx_buffer[0];
                sensor->distance = (float)raw_distance;
                sensor->data_ready = 1;
                sensor->state = US100_STATE_IDLE;
            }
            break;
            
        default:
            break;
    }
}

void US100_UART_RxCpltCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    for (uint8_t i = 0; i < us100_sensor_count; i++) {
        US100Sensor* s = active_sensors[i];
        
        if (huart == s->uart) {
            s->rx_index++;
            
            if (s->rx_index >= 2) {
                s->state = US100_STATE_RECEIVING;
            } else {
                HAL_UART_Receive_IT(huart, &s->rx_buffer[s->rx_index], 1);
            }
            
            break;
        }
    }
}

float US100_GetDistance(US100Sensor* sensor) {
    if (sensor->data_ready) {
        sensor->data_ready = 0;
        return sensor->distance;
    }
    return -1.0f; // 无效数据
}