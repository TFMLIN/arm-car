#include "work.h"
#include "trace.h"
#include "pca9685.h"
#include <stdio.h>

WorkState work; // 工作状态实例

void WorkInit(CarConfig *car, PCA9685_HandleTypeDef *pca)
{
    // 初始化工作状态
    work.mode    = 0; // 默认模式为0
    work.update  = 0; // 初始不更新
    work.RollX   = 0.0f;
    work.PitchY  = 0.0f;
    work.YawZ    = 0.0f;
    work.zRollx  = 0.0f;
    work.zPitchY = 0.0f;
    work.zYawZ   = 0.0f;
    work.car     = car; // 设置车辆配置
    work.pca     = pca;
}

// 模式0：待机，停止并矫正
void mode0()
{
    Car_Speed(work.car, 0, 0); // 停止车辆
    // 矫正车辆方向
    work.zYawZ = work.YawZ; // 更新偏航角为当前值
    PCA9685_SetServoPulse(work.pca->hi2c, 0, SERVO_PWM_MID);
}
// 模式1：直行
void mode1()
{
    Car_Speed(work.car, 100, 100);
    int trace_cnt       = 0;
    uint8_t trace_flag  = 0;
    uint8_t trace_state = 0;
    uint8_t i           = 0;
    uint8_t led         = 0;
    while (trace_flag == 0) {
        trace_state = Trace_ReadRegister();
        printf("Trace State: %02X\r\n", trace_state);
        if (trace_state != 0xff) {
            trace_cnt++;
            if (trace_cnt == 5) {
                trace_flag = 1; // 检测到轨迹
            }
        } else {
            trace_cnt = 0;
        }

        // 读取角度数据
        if (work.YawZ > 180) {
            work.YawZ -= 360;
        }
        int servo_pulse = SERVO_PWM_MID + (work.car->speed > 0 ? 1 : -1) * (int)(work.YawZ * 30); // 将YawZ转换为舵机脉冲宽度
        if (servo_pulse < 1000) servo_pulse = 1000;                                               // 限制最小脉冲宽度
        if (servo_pulse > 2000) servo_pulse = 2000;                                               // 限制最大脉冲宽度
        printf("YawZ: %.2f, Servo Pulse: %d\r\n", work.YawZ, servo_pulse);
        PCA9685_SetServoPulse(&hi2c1, 0, servo_pulse); // 设置舵机脉冲宽度

        i++;
        if (i == 10) {
            i   = 0;
            led = !led;
            HAL_GPIO_WritePin(GPIOB, LED3_Pin, led ? GPIO_PIN_SET : GPIO_PIN_RESET); // 切换LED状态
        }
        HAL_Delay(10);
    }
    Car_Speed(work.car, 0, 0);
}

// // 模式2：循迹转180°
// void mode2()
// {
//     Car_Speed(work.car, 100, 100);
//     int trace_cnt           = 0;
//     uint8_t trace_flag      = 0;
//     uint8_t trace_state     = 0;
//     uint8_t motor_left_base = 90, motor_right_base = 50;
//     uint8_t left_speed, right_speed;
//     int turn_pulse = SERVO_PWM_MID + 700;                   // 设定转向舵机脉冲宽度
//     PCA9685_SetServoPulse(&hi2c1, 0, turn_pulse);           // 设置舵机到转向位置
//     Car_Speed(work.car, motor_left_base, motor_right_base); // 启动车辆
//     uint8_t i   = 0;
//     uint8_t led = 0;
//     while (trace_flag == 0) {
//         trace_state = ~Trace_ReadRegister();
//         // trace_state &= 0xff;
//         printf("Trace State: %02X\r\n", trace_state);
//         // printf("Trace compare: %d\r\n", trace_state == 0xff);
//         if (trace_state == 0x00) {
//             trace_cnt++;
//             if (trace_cnt == 10) {
//                 trace_flag = 1; // 检测到轨迹
//             }
//         } else {
//             trace_cnt = 0;
//             // if (Trace_ReadRegister() == 0b111)
//             if (trace_state > 0x08) {
//                 left_speed = motor_left_base - 20, right_speed = motor_right_base + 20;
//             } else if (trace_state < 0x04) {
//                 left_speed = motor_left_base + 20, right_speed = motor_right_base - 50;
//             } else {
//                 left_speed = motor_left_base, right_speed = motor_right_base;
//             }
//             printf("left:%d right:%d\r\n", left_speed, right_speed);
//             Car_Speed(work.car, left_speed, right_speed);
//         }

//         i++;
//         if (i == 10) {
//             i   = 0;
//             led = !led;
//             HAL_GPIO_WritePin(GPIOB, LED3_Pin, led ? GPIO_PIN_SET : GPIO_PIN_RESET); // 切换LED状态
//         }
//         HAL_Delay(10);
//     }
//     HAL_Delay(500);
//     // 停止车辆并复位舵机
//     Car_Speed(work.car, 0, 0);
//     PCA9685_SetServoPulse(&hi2c1, 0, SERVO_PWM_MID); // 设置舵机到中位
// }

// 模式2：循迹转180°
void mode2()
{
    Car_Speed(work.car, 100, 100);
    int trace_cnt           = 0;
    uint8_t trace_flag      = 0;
    uint8_t trace_state     = 0;
    uint8_t motor_left_base = 120, motor_right_base = 60;
    uint8_t left_speed, right_speed;
    int turn_pulse = SERVO_PWM_MID + 700;                   // 设定转向舵机脉冲宽度
    PCA9685_SetServoPulse(&hi2c1, 0, turn_pulse);           // 设置舵机到转向位置
    Car_Speed(work.car, motor_left_base, motor_right_base); // 启动车辆
    uint8_t i   = 0;
    uint8_t led = 0;
    while (trace_flag == 0) {
        trace_state = ~Trace_ReadRegister();
        // trace_state &= 0xff;
        printf("Trace State: %02X\r\n", trace_state);
        // printf("Trace compare: %d\r\n", trace_state == 0xff);
        if (trace_state == 0x00) {
            trace_cnt++;
            if (trace_cnt == 10) {
                trace_flag = 1; // 检测到轨迹
            }
        } else {
            trace_cnt = 0;
            // if (Trace_ReadRegister() == 0b111)
            if (trace_state > 0x08) {
                left_speed = motor_left_base - 20, right_speed = motor_right_base + 20;
            } else if (trace_state < 0x04) {
                left_speed = motor_left_base + 20, right_speed = motor_right_base - 50;
            } else {
                left_speed = motor_left_base, right_speed = motor_right_base;
            }
            printf("left:%d right:%d\r\n", left_speed, right_speed);
            Car_Speed(work.car, left_speed, right_speed);
        }

        i++;
        if (i == 10) {
            i   = 0;
            led = !led;
            HAL_GPIO_WritePin(GPIOB, LED3_Pin, led ? GPIO_PIN_SET : GPIO_PIN_RESET); // 切换LED状态
        }
        HAL_Delay(10);
    }
    HAL_Delay(150);
    // 停止车辆并复位舵机
    Car_Speed(work.car, 0, 0);
    PCA9685_SetServoPulse(&hi2c1, 0, SERVO_PWM_MID); // 设置舵机到中位
}

void mode3()
{
}
void mode4()
{
}