#ifndef __WORK_H
#define __WORK_H

#include "stm32f4xx_hal.h"
#include "motor.h"
#include "servo.h"
#include "pca9685.h"

#define SERVO_PWM_MID 1500 // 中位脉宽

typedef struct {
    uint8_t mode;               // 工作模式
    uint8_t update;             // 更新标志
    float RollX;                // 车体滚转角
    float PitchY;               // 车体俯仰角
    float YawZ;                 // 车体偏航角
    float zRollx;               // 零点校正后的滚转角
    float zPitchY;              // 零点校正后的俯仰角
    float zYawZ;                // 零点校正后的偏航角
    CarConfig *car;             // 车辆配置
    PCA9685_HandleTypeDef *pca; // PCA9685句柄
} WorkState;

extern WorkState work; // 工作状态实例

void WorkInit(CarConfig *car, PCA9685_HandleTypeDef *pca);
void blink();
void straight_by_yaw_1(float target_yaw, uint8_t sub);
void straight_by_yaw_2(float target_yaw, uint8_t sub);

void mode0(); // 模式0：待机，停止并矫正
void mode1(); // 模式1：直行
void mode2();
void mode3();
void mode4();
void turn_right_time(int ms);

#endif // !__WORK_H