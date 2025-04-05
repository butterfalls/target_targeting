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
    .Kp = 3.0f,
    .Ki = 0.1f,
    .Kd = 0.2f,
    .integral = 0.0f,
    .prev_error = 0.0f,
    .max_integral = 100.0f
};

/* Exported functions --------------------------------------------------------*/
float PID_Calculate(PIDController* pid, float error, float dt) {
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