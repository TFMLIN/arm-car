#include "servo.h"

Servo servo; // Servo instance

void Servo_Init(TIM_HandleTypeDef *htim, uint32_t channel)
{
    servo.htim    = htim;    // Store the timer handle
    servo.channel = channel; // Store the PWM channel
    servo.angle   = 0.0f;    // Initialize angle to 0 degrees

    // Initialize the servo by setting the PWM channel
    HAL_TIM_PWM_Start(htim, channel);
    // Set the initial angle to 0 degrees
    Servo_SetAngle(htim, channel, 0.0f);
}

// 0 degree 0.5ms, 180 degree 2.5ms
// This function sets the angle of the servo motor by adjusting the PWM duty cycle
void Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t channel, float angle)
{
    servo.angle = angle; // Update the servo angle
    // Ensure the angle is within the valid range
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    // 定时器自动重装载值为10000
    uint32_t ARR = 10000;

    // 0度对应0.5ms，180度对应2.5ms
    float minMs = 0.5f;
    float maxMs = 2.5f;

    // 计算当前角度对应的高电平时间（ms）
    float pulseMs = minMs + (angle / 180.0f) * (maxMs - minMs);

    // 获取定时器时钟频率（Hz）
    uint32_t timerFreq = HAL_RCC_GetPCLK1Freq();
    if ((htim->Instance >= TIM2) && (htim->Instance <= TIM5)) {
        timerFreq *= 2; // APB1定时器时钟加倍
    }

    // 计算PWM周期（ms）
    float periodMs = (float)(ARR + 1) * (htim->Init.Prescaler + 1) / (float)timerFreq * 1000.0f;

    // 计算占空比
    float duty = pulseMs / periodMs;

    // 计算CCR值
    uint32_t pulseWidth = (uint32_t)(duty * (ARR + 1));

    // 设置PWM占空比
    __HAL_TIM_SET_COMPARE(htim, channel, pulseWidth);
}