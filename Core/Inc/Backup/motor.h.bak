#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "pid.h"

#define MOTOR_COUNT 2

struct PIDController;

// Motor struct
typedef struct {
    TIM_HandleTypeDef htim;
    uint32_t channel;
    uint8_t towards;
    GPIO_TypeDef *in_port1;
    GPIO_TypeDef *in_port2;
    uint16_t in_pin1;
    uint16_t in_pin2;
    TIM_HandleTypeDef en_htim;
    int16_t encoder_count;
    int16_t target_speed;
    PID_Controller pid;
} MotorConfig;

// Car struct
typedef struct {
    MotorConfig motor[2];
    int speed;
} CarConfig;

extern CarConfig car;

void Car_Speed_Percentage(CarConfig *car, int8_t p1, int8_t p2);
void Car_Speed(CarConfig *car, int s1, int s2);
void Car_Init(CarConfig *car);
void Car_Test(CarConfig *car);
int32_t Get_Encoder_Count(MotorConfig *motor);

void Motor_Speed_Control(CarConfig *car);

#ifdef __cplusplus
}
#endif

#endif // __MOTOR_H