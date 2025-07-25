#include "motor.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "main.h"
#include "stdio.h"

#define MAX_PWM   60000
#define MAX_SPEED 300

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;

CarConfig car;

// Motor start
void Motor_Start(MotorConfig *motor)
{
    HAL_TIM_PWM_Start(&motor->htim, motor->channel);
    HAL_TIM_Encoder_Start(&motor->en_htim, TIM_CHANNEL_ALL);
    __HAL_TIM_CLEAR_FLAG(&motor->en_htim, TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start_IT(&motor->en_htim);
}

// Car Init
void Car_Init(CarConfig *car)
{
    car->motor[0] = (MotorConfig){htim1, TIM_CHANNEL_1, 1, MOTOR1_IN1_GPIO_Port, MOTOR1_IN2_GPIO_Port, MOTOR1_IN1_Pin, MOTOR1_IN2_Pin, htim2, 0};
    car->motor[1] = (MotorConfig){htim1, TIM_CHANNEL_4, 0, MOTOR2_IN1_GPIO_Port, MOTOR2_IN2_GPIO_Port, MOTOR2_IN1_Pin, MOTOR2_IN2_Pin, htim3, 0};
    for (int i = 0; i < MOTOR_COUNT; i++) {
        PID_Init(&car->motor[i].pid, 200, 800, 0.13, 0, MAX_PWM);
    }
    for (int i = 0; i < MOTOR_COUNT; i++) {
        __HAL_TIM_SET_COUNTER(&car->motor[i].en_htim, 0);
    }
    for (int i = 0; i < MOTOR_COUNT; i++) {
        Motor_Start(&car->motor[i]);
    }
    HAL_TIM_Base_Start_IT(&htim6);
}

// Motor speed control
void Motor_Speed_PWM(MotorConfig *motor, int speed)
{
    if (speed > MAX_PWM)
        speed = MAX_PWM;
    else if (speed < -MAX_PWM)
        speed = -MAX_PWM;
    if (speed == 0) {
        HAL_GPIO_WritePin(motor->in_port1, motor->in_pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->in_port2, motor->in_pin2, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&motor->htim, motor->channel, 0);
        HAL_TIM_PWM_Stop(&motor->htim, motor->channel);
        return;
    }
    uint8_t direction = speed < 0;
    if (direction) speed = -speed;
    // 根据安装方向的不同设置电机转向
    if (direction ^ motor->towards) {
        HAL_GPIO_WritePin(motor->in_port1, motor->in_pin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->in_port2, motor->in_pin2, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(motor->in_port1, motor->in_pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->in_port2, motor->in_pin2, GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&motor->htim, motor->channel, speed);
    HAL_TIM_PWM_Start(&motor->htim, motor->channel);
}

void Car_Speed_PWM(CarConfig *car, int s1, int s2)
{
    Motor_Speed_PWM(&car->motor[0], s1);
    Motor_Speed_PWM(&car->motor[1], s2);
}

void Motor_Speed(MotorConfig *motor, int speed)
{
    if (speed == 0) PID_Reset(&motor->pid);
    if (speed > MAX_SPEED)
        speed = MAX_SPEED;
    else if (speed < -MAX_SPEED)
        speed = -MAX_SPEED;
    motor->target_speed = speed;
}

void Car_Speed_Percentage(CarConfig *car, int8_t p1, int8_t p2)
{
    int s1 = (p1 * MAX_SPEED) / 100;
    int s2 = (p2 * MAX_SPEED) / 100;
    Car_Speed(car, p1, p2);
}

void Car_Speed(CarConfig *car, int s1, int s2)
{
    car->speed = (s1 + s2) / 2;
    Motor_Speed(&car->motor[0], s1);
    Motor_Speed(&car->motor[1], s2);
}

void Car_Test(CarConfig *car)
{
}

// 示例使用
void Motor_Speed_Control(CarConfig *car)
{
    uint8_t i;
    for (i = 0; i < MOTOR_COUNT; i++) {
        car->motor[i].pid.setpoint = car->motor[i].target_speed;
    }

    // 读取编码器计数值
    for (i = 0; i < MOTOR_COUNT; i++) {
        if (car->motor[i].towards) {
            car->motor[i].encoder_count = Get_Encoder_Count(&car->motor[i]);
        } else {
            car->motor[i].encoder_count = -Get_Encoder_Count(&car->motor[i]);
        }
    }

    printf("Speed:%d,%d,%d,%d\r\n", car->motor[0].encoder_count, car->motor[1].encoder_count,
           car->motor[0].target_speed, car->motor[1].target_speed);

    // 计算 PID 控制量
    for (i = 0; i < MOTOR_COUNT; i++) {
        float dt      = 0.1f;
        float control = PID_Compute(&car->motor[i].pid, car->motor[i].encoder_count, dt);
        printf("Control:%f\r\n", control);
        Motor_Speed_PWM(&car->motor[i], control);
    }

    for (i = 0; i < MOTOR_COUNT; i++) {
        __HAL_TIM_SET_COUNTER(&car->motor[i].en_htim, 0);
    }
}
