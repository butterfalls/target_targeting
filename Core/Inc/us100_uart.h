#ifndef US100_UART_H
#define US100_UART_H

#include "stm32f4xx_hal.h"
#include <math.h>

// US-100串口命令
#define US100_CMD_READ_DISTANCE 0x55  // 读取距离命令

// 最大支持的传感器数量
#define MAX_US100_SENSORS 5

// 卡尔曼滤波器结构体
typedef struct {
    float x;      // 状态估计值
    float P;      // 估计误差协方差
    float Q;      // 过程噪声协方差
    float R;      // 测量噪声协方差
    float K;      // 卡尔曼增益
    float dt;     // 时间步长
} KalmanFilter;

// 滑动窗口滤波器结构体
typedef struct {
    float* buffer;    // 数据缓冲区
    int size;         // 窗口大小
    int index;        // 当前索引
    float sum;        // 数据总和
} SlidingWindowFilter;

// 传感器状态
typedef enum {
    US100_STATE_IDLE,       // 空闲状态
    US100_STATE_SENDING,    // 发送命令状态
    US100_STATE_WAITING,    // 等待响应状态
    US100_STATE_RECEIVING   // 接收数据状态
} US100State;

// 传感器结构体
typedef struct {
    // 硬件接口
    UART_HandleTypeDef* uart;  // 串口句柄
    
    // 状态变量
    US100State state;          // 当前状态
    uint32_t timestamp;        // 时间戳
    uint8_t rx_buffer[2];      // 接收缓冲区
    uint8_t rx_index;          // 接收索引
    
    // 测量结果
    uint16_t distance;         // 距离值
    uint8_t data_ready;        // 数据就绪标志
    
    // 滤波器
    KalmanFilter kalman;       // 卡尔曼滤波器
    SlidingWindowFilter sliding; // 滑动窗口滤波器
} US100Sensor;

// 全局变量
extern uint8_t us100_sensor_count;  // 当前活动的传感器数量
extern US100Sensor* active_sensors[MAX_US100_SENSORS];  // 活动传感器数组
extern float last_valid_distances[MAX_US100_SENSORS];  // 上次有效的距离值
extern float raw_distances[4];

// 初始化传感器
void US100_Init(US100Sensor* sensor, UART_HandleTypeDef* uart);

// 开始一次新的测量
void US100_StartMeasurement(US100Sensor* sensor);

// 状态机更新，需要定期调用
void US100_Update(US100Sensor* sensor);

// 获取最新测量结果
float US100_GetDistance(US100Sensor* sensor);

// 串口接收回调函数
void US100_UART_RxCpltCallback(UART_HandleTypeDef *huart);

// 获取所有传感器的有效距离值
void US100_GetAllValidDistances(float* distances);

// 卡尔曼滤波器函数
void KalmanFilter_Init(KalmanFilter* kf, float Q, float R, float dt);
float KalmanFilter_Update(KalmanFilter* kf, float measurement);

// 滑动窗口滤波器函数
void SlidingWindowFilter_Init(SlidingWindowFilter* swf, int size);
float SlidingWindowFilter_Update(SlidingWindowFilter* swf, float new_value);
void SlidingWindowFilter_Reset(SlidingWindowFilter* swf);

#endif /* US100_UART_H */

/*使用时在开头init startmeasure rxcplt修改，while里只需要updateall*/