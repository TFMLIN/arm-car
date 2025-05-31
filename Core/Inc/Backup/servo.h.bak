#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f4xx_hal.h"
#include "tim.h"

struct Servo {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    float angle; // Angle in degrees
};

void Servo_Init(TIM_HandleTypeDef *htim, uint32_t channel);
void Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t channel, float angle);

#endif // !__SERVO_H