#include "stm32f4xx_hal.h"
#include "motor.h"
#include "pid.h"
#include "stdio.h"

// 初始化 PID 控制器
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float setpoint, float max_control)
{
    pid->kp             = kp;
    pid->ki             = ki;
    pid->kd             = kd;
    pid->setpoint       = setpoint;
    pid->integral       = 0.0f;
    pid->previous_error = 0.0f;
    pid->max_control    = max_control;
}

void PID_Reset(PID_Controller *pid)
{
    pid->integral       = 0.0f;
    pid->previous_error = 0.0f;
}

// PID 控制算法
float PID_Compute(PID_Controller *pid, float measured_value, float dt)
{
    float error = pid->setpoint - measured_value;
    // printf("Error:%f\r\n", error);
    if (error < 120) {
        pid->integral += error * dt;
    }
    float derivative    = (error - pid->previous_error) / dt;
    float output        = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    pid->previous_error = error;
    // 限制输出范围
    if (output > pid->max_control) {
        output = pid->max_control;
    } else if (output < -pid->max_control) {
        output = -pid->max_control;
    }
    return output;
}

// 获取编码器计数值
int32_t Get_Encoder_Count(MotorConfig *motor)
{
    int32_t cnt = __HAL_TIM_GET_COUNTER(&motor->en_htim);
    return cnt;
}
