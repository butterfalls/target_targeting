#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>

/* Exported types ------------------------------------------------------------*/
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float max_integral;
} PIDController;

/* Exported variables -------------------------------------------------------*/
extern PIDController pid_encoder;  // 编码器PID控制器
extern PIDController pid_yaw;      // 偏航角PID控制器
extern PIDController pid_front;    // 前轮组同步PID控制器
extern PIDController pid_rear;     // 后轮组同步PID控制器
extern PIDController pid_position; // 左右轮组同步PID控制器

/* Exported functions prototypes ---------------------------------------------*/
float PID_Calculate(PIDController* pid, float error, float dt);
void PID_Reset(PIDController* pid);

#ifdef __cplusplus
}
#endif

#endif /* __PID_CONTROLLER_H */ 