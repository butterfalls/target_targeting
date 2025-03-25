#ifndef ULTRASONIC_NONBLOCKING_H
#define ULTRASONIC_NONBLOCKING_H

#include "stm32f4xx_hal.h"

typedef enum {
    US_STATE_IDLE,          // ����״̬
    US_STATE_TRIG_START,    // ������ʼ
    US_STATE_TRIG_PULSE,    // ����������
    US_STATE_WAIT_ECHO      // �ȴ��ز�
} UltrasonicState;

typedef struct {
    // Ӳ���ӿ�
    GPIO_TypeDef* trig_port;
    uint16_t trig_pin;
    GPIO_TypeDef* echo_port;
    uint16_t echo_pin;
    
    // ״̬����
    UltrasonicState state;
    uint32_t timestamp;     // ����SysTick��ʱ���
    uint32_t pulse_start;   // �ز���ʼʱ��
    
    // �������
    float distance;
    uint8_t data_ready;
} UltrasonicSensor;

// ��ʼ��������
void Ultrasonic_Init(UltrasonicSensor* sensor);

// ��������������
void Ultrasonic_StartMeasurement(UltrasonicSensor* sensor);

// ��״̬�����£��������Ե��ã�
void Ultrasonic_Update(UltrasonicSensor* sensor);

// ��ȡ���²������
float Ultrasonic_GetDistance(UltrasonicSensor* sensor);

#endif