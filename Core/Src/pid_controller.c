#include "pid_controller.h"

/* Private variables ---------------------------------------------------------*/
PIDController pid_encoder = {
    .Kp = 3.2f,
    .Ki = 0.1f,
    .Kd = 0.4f,
    .integral = 0.0f,
    .prev_error = 0.0f,
    .max_integral = 100.0f
};

PIDController pid_yaw = {
    .Kp = 2.0f,    // 降低比例系数
    .Ki = 0.05f,   // 降低积分系数
    .Kd = 0.1f,    // 降低微分系数
    .integral = 0.0f,
    .prev_error = 0.0f,
    .max_integral = 50.0f  // 降低积分上限
};

PIDController pid_front = {
    .Kp = 2.0f,
    .Ki = 0.05f,
    .Kd = 0.1f,
    .integral = 0.0f,
    .prev_error = 0.0f,
    .max_integral = 50.0f
};

PIDController pid_rear = {
    .Kp = 2.0f,
    .Ki = 0.05f,
    .Kd = 0.1f,
    .integral = 0.0f,
    .prev_error = 0.0f,
    .max_integral = 50.0f
};

PIDController pid_position = {
    .Kp = 2.0f,
    .Ki = 0.05f,
    .Kd = 0.1f,
    .integral = 0.0f,
    .prev_error = 0.0f,
    .max_integral = 50.0f
};

/* Exported functions --------------------------------------------------------*/
float PID_Calculate(PIDController* pid, float error, float dt) {
    // 添加时间差保护
    if (dt <= 0.001f) {
        dt = 0.001f;  // 最小时间差为1ms
    }

    float proportional = pid->Kp * error;

    pid->integral += error * dt;
    pid->integral = fmaxf(fminf(pid->integral, pid->max_integral), -pid->max_integral);

    float derivative = pid->Kd * (error - pid->prev_error) / dt;

    float output = proportional + (pid->Ki * pid->integral) + derivative;

    pid->prev_error = error;

    return output;
}

void PID_Reset(PIDController* pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
} 