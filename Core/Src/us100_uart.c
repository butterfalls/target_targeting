#include "us100_uart.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

// 全局传感器数组
static US100Sensor* active_sensors[MAX_US100_SENSORS] = {0};
uint8_t us100_sensor_count = 0;  // 当前活动的传感器数量

// 超时时间（毫秒）
#define US100_TIMEOUT_MS 300  // 增加超时时间到300ms

// 静态变量用于存储上次有效的距离值
static float last_valid_distances[MAX_US100_SENSORS] = {0};

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
                // 检查数据有效性
                if (sensor->rx_buffer[0] == 0xFF && sensor->rx_buffer[1] == 0xFF) {
                    sensor->state = US100_STATE_IDLE;
                    sensor->rx_index = 0;
                    HAL_UART_Receive_IT(sensor->uart, &sensor->rx_buffer[0], 1);
                    return;
                }
                
                // 计算距离：低字节在前，高字节在后
                uint16_t raw_distance = (sensor->rx_buffer[1] << 8) | sensor->rx_buffer[0];
                
                // 检查距离值是否在合理范围内（例如0-4000mm）
                if (raw_distance > 40000) {
                    // 距离值超出范围，视为无效数据
                    sensor->state = US100_STATE_IDLE;
                    sensor->rx_index = 0;
                    HAL_UART_Receive_IT(sensor->uart, &sensor->rx_buffer[0], 1);
                    return;
                }
                
                sensor->distance = raw_distance;
                sensor->data_ready = 1;
                sensor->state = US100_STATE_IDLE;
            }
            break;
            
        default:
            break;
    }
}

void US100_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
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

void US100_GetAllValidDistances(float* distances) {
    // 更新所有传感器的状态
    for (uint8_t i = 0; i < us100_sensor_count; i++) {
        US100_Update(active_sensors[i]);
    }
    
    // 获取所有传感器的距离值
    for (uint8_t i = 0; i < us100_sensor_count; i++) {
        float current_distance = US100_GetDistance(active_sensors[i]);
        if (current_distance > 0) {
            last_valid_distances[i] = current_distance;
        }
        distances[i] = last_valid_distances[i];
    }
    
    // 检查是否所有传感器都有有效数据
    uint8_t all_valid = 1;
    for (uint8_t i = 0; i < us100_sensor_count; i++) {
        if (distances[i] <= 0) {
            all_valid = 0;
            break;
        }
    }
    
    // 如果所有传感器都有有效数据，开始下一次测量
    if (all_valid) {
        for (uint8_t i = 0; i < us100_sensor_count; i++) {
            US100_StartMeasurement(active_sensors[i]);
        }
    }
    // 如果所有传感器都没有有效数据，检查是否需要重新测量
    else {
        static uint32_t last_measurement_time = 0;
        static uint8_t timeout_count = 0;
        
        uint32_t current_time = HAL_GetTick();
        
        if (current_time - last_measurement_time > 30) {
            timeout_count++;
            
            if (timeout_count >= 3) {
                for (uint8_t i = 0; i < us100_sensor_count; i++) {
                    US100_StartMeasurement(active_sensors[i]);
                }
                timeout_count = 0;
            }
            
            last_measurement_time = current_time;
        }
    }
}

/*--------------------------------------------------使用实例------------------------------------------------------------*/
    // static float last_valid_distance1 = 0;
    // static float last_valid_distance2 = 0;
    // static float last_valid_distance3 = 0;
    // static float last_valid_distance4 = 0;

    // US100Sensor us100_sensor1;  // US100传感器实例
    // US100Sensor us100_sensor2;  // US100传感器实例
    // US100Sensor us100_sensor3;  // US100传感器实例
    // US100Sensor us100_sensor4;  // US100传感器实例



    // US100_StartMeasurement(&us100_sensor1);
    // US100_StartMeasurement(&us100_sensor2);
    // US100_StartMeasurement(&us100_sensor3);
    // US100_StartMeasurement(&us100_sensor4);


    // US100_Update(&us100_sensor1);
    // US100_Update(&us100_sensor2);
    // US100_Update(&us100_sensor3);
    // US100_Update(&us100_sensor4);
        
    // float us100_distance1 = US100_GetDistance(&us100_sensor1);
    // float us100_distance2 = US100_GetDistance(&us100_sensor2);
    // float us100_distance3 = US100_GetDistance(&us100_sensor3);
    // float us100_distance4 = US100_GetDistance(&us100_sensor4);
    
    // if (us100_distance1 > 0) last_valid_distance1 = us100_distance1;
    // if (us100_distance2 > 0) last_valid_distance2 = us100_distance2;
    // if (us100_distance3 > 0) last_valid_distance3 = us100_distance3;
    // if (us100_distance4 > 0) last_valid_distance4 = us100_distance4;
    
    // if (last_valid_distance1 > 0 && last_valid_distance2 > 0 && last_valid_distance3 > 0 && last_valid_distance4 > 0) {
    //     char buf[128];
    //     sprintf(buf, "US100: %.0f, %.0f, %.0f, %.0f mm\r\n", 
    //             last_valid_distance1, last_valid_distance2, last_valid_distance3, last_valid_distance4);
    //     HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
        
    //     // 开始下一次测量
    //     US100_StartMeasurement(&us100_sensor1);
    //     US100_StartMeasurement(&us100_sensor2);
    //     US100_StartMeasurement(&us100_sensor3);
    //     US100_StartMeasurement(&us100_sensor4);
    // } 
    // else if ((us100_distance1 == 0||us100_distance1 == -1) && (us100_distance2 == 0||us100_distance2 == -1) && 
    //          (us100_distance3 == 0||us100_distance3 == -1) && (us100_distance4 == 0||us100_distance4 == -1)){
    //     static uint32_t last_measurement_time = 0;
    //     static uint8_t timeout_count = 0;
        
    //     uint32_t current_time = HAL_GetTick();
        
    //     if (current_time - last_measurement_time > 300) {
    //         timeout_count++;
            
    //         // 输出调试信息
    //         char debug_buf[64];
    //         sprintf(debug_buf, "US100: No valid data for %d times\r\n", timeout_count);
    //         HAL_UART_Transmit(&huart1, (uint8_t*)debug_buf, strlen(debug_buf), 100);
            
    //         if (timeout_count >= 20) {
    //             US100_StartMeasurement(&us100_sensor1);
    //             US100_StartMeasurement(&us100_sensor2);
    //             US100_StartMeasurement(&us100_sensor3);
    //             US100_StartMeasurement(&us100_sensor4);
    //             timeout_count = 0;
    //         }
            
    //         last_measurement_time = current_time;
    //     }
    // }
