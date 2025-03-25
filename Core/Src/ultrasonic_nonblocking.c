#include "ultrasonic_nonblocking.h"

// SysTickʱ���׼��1ms�ֱ��ʣ�
#define SYSTICK_FREQ 1000

// ���������ȣ���s��
#define TRIG_PULSE_WIDTH 20

void Ultrasonic_Init(UltrasonicSensor* sensor) {
    // ����TrigΪ���
    GPIO_InitTypeDef gpio_init = {
        .Pin = sensor->trig_pin,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH
    };
    HAL_GPIO_Init(sensor->trig_port, &gpio_init);
    
    // ����EchoΪ�ж�����
    gpio_init.Pin = sensor->echo_pin;
    gpio_init.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio_init.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(sensor->echo_port, &gpio_init);
    
    sensor->state = US_STATE_IDLE;
    sensor->data_ready = 0;
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
            // ������ʼ������TrigΪ�͵�ƽ
            HAL_GPIO_WritePin(sensor->trig_port, sensor->trig_pin, GPIO_PIN_RESET);
            sensor->timestamp = now;
            sensor->state = US_STATE_TRIG_PULSE;
            break;
            
        case US_STATE_TRIG_PULSE:
            // ��������ά�ֽ׶�
            if ((now - sensor->timestamp) >= 2) { // �ȴ�2ms�͵�ƽ
                HAL_GPIO_WritePin(sensor->trig_port, sensor->trig_pin, GPIO_PIN_SET);
                sensor->timestamp = now;
                sensor->state = US_STATE_WAIT_ECHO;
            }
            break;
            
        case US_STATE_WAIT_ECHO:
            // ά�ִ���������
            if ((now - sensor->timestamp) * 1000 >= TRIG_PULSE_WIDTH) { // ת��Ϊ��s
                HAL_GPIO_WritePin(sensor->trig_port, sensor->trig_pin, GPIO_PIN_RESET);
                sensor->state = US_STATE_IDLE;
            }
            break;
            
        default:
            break;
    }
}

// Echo�жϻص�
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // �������д���������ƥ���Echo����
    extern UltrasonicSensor* active_sensors[]; // �û�����Ӧ�ò㶨��
    
    for (uint8_t i=0; active_sensors[i]; i++) {
        UltrasonicSensor* s = active_sensors[i];
        
        if (GPIO_Pin == s->echo_pin) {
            if (HAL_GPIO_ReadPin(s->echo_port, s->echo_pin)) {
                // �����أ���¼��ʼʱ��
                s->pulse_start = HAL_GetTick();
            } else {
                // �½��أ��������ʱ��
                uint32_t duration = HAL_GetTick() - s->pulse_start;
                s->distance = (duration * 34300.0f) / (2 * SYSTICK_FREQ); // ��λ������
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
    return -1.0f; // ��Ч����
}