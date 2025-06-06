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

void blink()
{
    static uint8_t i = 0, led = 0;
    i++;
    if (i == 10) {
        i   = 0;
        led = !led;
        HAL_GPIO_WritePin(GPIOB, LED3_Pin, led ? GPIO_PIN_SET : GPIO_PIN_RESET); // 切换LED状态
    }
}

// 模式0：待机，停止并矫正
void mode0()
{
    Car_Speed(work.car, 0, 0); // 停止车辆
    // 矫正车辆方向
    work.zYawZ = work.YawZ; // 更新偏航角为当前值
    PCA9685_SetServoPulse(work.pca->hi2c, 0, SERVO_PWM_MID);
}

// 模式1：0度直行
void mode1()
{
    printf("begin yaw:%.2f\r\n", work.YawZ);

    Car_Speed(work.car, 100, 100);
    int trace_cnt       = 0;
    uint8_t trace_flag  = 0;
    uint8_t trace_state = 0;
    uint8_t i           = 0;
    uint8_t led         = 0;
    while (trace_flag == 0) {
        trace_state = Trace_ReadRegister();
        // printf("Trace State: %02X\r\n", trace_state);
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
        // printf("YawZ: %.2f, Servo Pulse: %d\r\n", work.YawZ, servo_pulse);
        PCA9685_SetServoPulse(&hi2c1, 0, servo_pulse); // 设置舵机脉冲宽度

        blink();
        HAL_Delay(10);
    }
    Car_Speed(work.car, 0, 0);
    printf("end yaw:%.2f\r\n", work.YawZ);
}

// 模式2：右转循迹转180°
void mode2()
{
    printf("begin yaw:%.2f\r\n", work.YawZ);

    Car_Speed(work.car, 0, 0);
    int trace_cnt           = 0;
    uint8_t trace_flag      = 0;
    uint8_t trace_state     = 0;
    uint8_t motor_left_base = 90, motor_right_base = 40;
    uint8_t left_speed, right_speed;
    int turn_pulse = SERVO_PWM_MID + 750;                   // 设定转向舵机脉冲宽度
    PCA9685_SetServoPulse(&hi2c1, 0, turn_pulse);           // 设置舵机到转向位置
    Car_Speed(work.car, motor_left_base, motor_right_base); // 启动车辆
    while (trace_flag == 0) {
        trace_state = ~Trace_ReadRegister();
        // trace_state &= 0xff;
        // printf("Trace State: %02X\r\n", trace_state);
        // printf("Trace compare: %d\r\n", trace_state == 0xff);
        if (trace_state == 0x00) {
            trace_cnt++;
            if (trace_cnt == 30) {
                trace_flag = 1; // 检测到轨迹
            }
        } else {
            trace_cnt = 0;
            if (trace_state > 0x20) {
                PCA9685_SetServoPulse(&hi2c1, 0, turn_pulse - 500); // 设置舵机到转向位置
                left_speed = motor_left_base - 30, right_speed = motor_right_base + 20;
            } else if (trace_state > 0x10) {
                // PCA9685_SetServoPulse(&hi2c1, 0, turn_pulsen - 100);           // 设置舵机到转向位置
                // left_speed = motor_left_base - 20, right_speed = motor_right_base + 20;
                PCA9685_SetServoPulse(&hi2c1, 0, turn_pulse); // 设置舵机到转向位置
                left_speed = motor_left_base - 30, right_speed = motor_right_base + 30;
            } else if (trace_state > 0x04) {
                PCA9685_SetServoPulse(&hi2c1, 0, turn_pulse); // 设置舵机到转向位置
                left_speed = motor_left_base - 20, right_speed = motor_right_base + 20;
            } else if (trace_state < 0x04) {
                PCA9685_SetServoPulse(&hi2c1, 0, turn_pulse); // 设置舵机到转向位置
                left_speed = motor_left_base + 30, right_speed = motor_right_base - 30;
            } else {
                PCA9685_SetServoPulse(&hi2c1, 0, turn_pulse); // 设置舵机到转向位置
                left_speed = motor_left_base, right_speed = motor_right_base;
            }
            // printf("left:%d right:%d\r\n", left_speed, right_speed);
            Car_Speed(work.car, left_speed, right_speed);
        }

        blink();
        HAL_Delay(10);
    }
    Car_Speed(work.car, 0, 0);
    PCA9685_SetServoPulse(&hi2c1, 0, SERVO_PWM_MID + 400);
    HAL_Delay(50);
    Car_Speed(work.car, 80, 20);
    HAL_Delay(800);
    // HAL_Delay(200);
    // 停止车辆并复位舵机
    Car_Speed(work.car, 0, 0);
    PCA9685_SetServoPulse(&hi2c1, 0, SERVO_PWM_MID); // 设置舵机到中位
    printf("end yaw:%.2f\r\n", work.YawZ);
}

// 模式3：左转循迹180度
void mode3()
{
    printf("begin yaw:%.2f\r\n", work.YawZ);

    Car_Speed(work.car, 0, 0);
    int trace_cnt           = 0;
    uint8_t trace_flag      = 0;
    uint8_t trace_state     = 0;
    uint8_t motor_left_base = 40, motor_right_base = 90; // 40 90
    uint8_t left_speed, right_speed;
    int turn_pulse = SERVO_PWM_MID - 750;                   // 750                   // 设定转向舵机脉冲宽度
    PCA9685_SetServoPulse(&hi2c1, 0, turn_pulse);           // 设置舵机到转向位置
    Car_Speed(work.car, motor_left_base, motor_right_base); // 启动车辆
    while (trace_flag == 0) {
        trace_state = ~Trace_ReadRegister();
        // trace_state &= 0xff;
        // printf("Trace State: %02X\r\n", trace_state);
        // printf("Trace compare: %d\r\n", trace_state == 0xff);
        if (trace_state == 0x00) {
            trace_cnt++;
            if (trace_cnt == 30) {
                trace_flag = 1; // 检测到轨迹
            }
        } else {
            trace_cnt = 0;
            if (trace_state < 0x04) {
                PCA9685_SetServoPulse(&hi2c1, 0, turn_pulse + 500); // 设置舵机到转向位置
                left_speed = motor_left_base + 20, right_speed = motor_right_base - 30;
            } else if (trace_state < 0x08) {
                // PCA9685_SetServoPulse(&hi2c1, 0, turn_pulsen - 100);           // 设置舵机到转向位置
                // left_speed = motor_left_base - 20, right_speed = motor_right_base + 20;
                PCA9685_SetServoPulse(&hi2c1, 0, turn_pulse); // 设置舵机到转向位置
                left_speed = motor_left_base + 30, right_speed = motor_right_base - 30;
            } else if (trace_state < 0x20) {
                PCA9685_SetServoPulse(&hi2c1, 0, turn_pulse); // 设置舵机到转向位置
                left_speed = motor_left_base + 20, right_speed = motor_right_base - 20;
            } else if (trace_state > 0x04) {
                PCA9685_SetServoPulse(&hi2c1, 0, turn_pulse); // 设置舵机到转向位置
                left_speed = motor_left_base - 30, right_speed = motor_right_base + 30;
            } else {
                PCA9685_SetServoPulse(&hi2c1, 0, turn_pulse); // 设置舵机到转向位置
                left_speed = motor_left_base, right_speed = motor_right_base;
            }
            // printf("left:%d right:%d\r\n", left_speed, right_speed);
            Car_Speed(work.car, left_speed, right_speed);
        }

        blink();
        HAL_Delay(10);
    }
    Car_Speed(work.car, 0, 0);
    PCA9685_SetServoPulse(&hi2c1, 0, SERVO_PWM_MID - 400);
    HAL_Delay(50);               // 50
    Car_Speed(work.car, 20, 80); // 20  80
    HAL_Delay(800);
    // HAL_Delay(200);
    // 停止车辆并复位舵机
    Car_Speed(work.car, 0, 0);
    PCA9685_SetServoPulse(&hi2c1, 0, SERVO_PWM_MID); // 设置舵机到中位
    printf("end yaw:%.2f\r\n", work.YawZ);
}

// 模式4：141.35度直行
void mode4()
{
    printf("yaw:%.2f\r\n", work.YawZ);

    Car_Speed(work.car, 0, 0);
    int trace_cnt           = 0;
    uint8_t trace_flag      = 0;
    uint8_t trace_state     = 0;
    uint8_t motor_left_base = 80, motor_right_base = 80;
    int turn_pulse = SERVO_PWM_MID;                         // 设定转向舵机脉冲宽度
    PCA9685_SetServoPulse(&hi2c1, 0, turn_pulse);           // 设置舵机到转向位置
    Car_Speed(work.car, motor_left_base, motor_right_base); // 启动车辆
    float target_yaw = 141.35;
    // float target_yaw = 145.35;
    while (trace_flag == 0) {
        // printf("")
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

        int servo_pulse = SERVO_PWM_MID + (work.car->speed > 0 ? 1 : -1) * (int)((work.YawZ - target_yaw) * 40); // 将YawZ转换为舵机脉冲宽度
        if (servo_pulse < 800) servo_pulse = 800;                                                                // 限制最小脉冲宽度
        if (servo_pulse > 2200) servo_pulse = 2200;                                                              // 限制最大脉冲宽度
        printf("YawZ: %.2f, Servo Pulse: %d\r\n", work.YawZ, servo_pulse);
        PCA9685_SetServoPulse(&hi2c1, 0, servo_pulse); // 设置舵机脉冲宽度

        blink();
        HAL_Delay(10);
    }
    Car_Speed(work.car, 0, 0);
}
int car_lenght = 0;

// target_yaw是目标角度，sub为1则是-180~180模式，sub为0则是0~360模式
void straight_by_yaw_1(float target_yaw, uint8_t sub)
{
    printf("begin yaw:%.2f\r\n", work.YawZ);

    Car_Speed(work.car, 0, 0);
    int trace_cnt           = 0;
    uint8_t trace_flag      = 0;
    uint8_t trace_state     = 0;
    uint8_t motor_left_base = 60, motor_right_base = 60;
    int turn_pulse = SERVO_PWM_MID;                         // 设定转向舵机脉冲宽度
    PCA9685_SetServoPulse(&hi2c1, 0, turn_pulse);           // 设置舵机到转向位置
    Car_Speed(work.car, motor_left_base, motor_right_base); // 启动车辆

    while (trace_flag == 0) {
        car_lenght++;
        // printf("")
        trace_state = Trace_ReadRegister();
        // printf("Trace State: %02X\r\n", trace_state);
        if (trace_state != 0xff) {
            trace_cnt++;
            if (trace_cnt == 5) {
                car_lenght = 0;
                trace_flag = 1; // 检测到轨迹
            }
        } else {
            trace_cnt = 0;
        }
        if (sub && work.YawZ > 180) work.YawZ -= 360;
        if (car_lenght <= 350) {
            int servo_pulse = SERVO_PWM_MID + (work.car->speed > 0 ? 1 : -1) * (int)((work.YawZ - target_yaw) * 40); // 将YawZ转换为舵机脉冲宽度
            if (servo_pulse < 800) servo_pulse = 800;                                                                // 限制最小脉冲宽度
            if (servo_pulse > 2200) servo_pulse = 2200;                                                              // 限制最大脉冲宽度
            // printf("YawZ: %.2f, Servo Pulse: %d\r\n", work.YawZ, servo_pulse);
            PCA9685_SetServoPulse(&hi2c1, 0, servo_pulse); // 设置舵机脉冲宽度
        } else {
            PCA9685_SetServoPulse(&hi2c1, 0, SERVO_PWM_MID - 200);
        }
        // 读取角度数据

        blink();
        HAL_Delay(10);
    }
    car_lenght = 0;
    Car_Speed(work.car, 0, 0);
    printf("end yaw:%.2f\r\n", work.YawZ);
}

// target_yaw是目标角度，sub为1则是-180~180模式，sub为0则是0~360模式
void straight_by_yaw_2(float target_yaw, uint8_t sub)
{
    printf("begin yaw:%.2f\r\n", work.YawZ);

    Car_Speed(work.car, 0, 0);
    int trace_cnt           = 0;
    uint8_t trace_flag      = 0;
    uint8_t trace_state     = 0;
    uint8_t motor_left_base = 60, motor_right_base = 60;
    int turn_pulse = SERVO_PWM_MID;                         // 设定转向舵机脉冲宽度
    PCA9685_SetServoPulse(&hi2c1, 0, turn_pulse);           // 设置舵机到转向位置
    Car_Speed(work.car, motor_left_base, motor_right_base); // 启动车辆
    int car = 0;
    while (trace_flag == 0) {
        car++;
        // printf("")
        trace_state = Trace_ReadRegister();
        // printf("Trace State: %02X\r\n", trace_state);
        if (trace_state != 0xff) {
            trace_cnt++;
            if (trace_cnt == 5) {
                trace_flag = 1; // 检测到轨迹
            }
        } else {
            trace_cnt = 0;
        }
        if (sub && work.YawZ > 180) work.YawZ -= 360;
        if (car <= 300) {
            int servo_pulse = SERVO_PWM_MID + (work.car->speed > 0 ? 1 : -1) * (int)((work.YawZ - target_yaw) * 40); // 将YawZ转换为舵机脉冲宽度
            if (servo_pulse < 1000) servo_pulse = 1000;                                                              // 限制最小脉冲宽度
            if (servo_pulse > 2000) servo_pulse = 2000;                                                              // 限制最大脉冲宽度
            // printf("YawZ: %.2f, Servo Pulse: %d\r\n", work.YawZ, servo_pulse);
            PCA9685_SetServoPulse(&hi2c1, 0, servo_pulse); // 设置舵机脉冲宽度
        } else {
            PCA9685_SetServoPulse(&hi2c1, 0, SERVO_PWM_MID + 280);
        }
        //     // 读取角度数据
        printf("yaw:%.2f, %.2f\r\n", work.YawZ, target_yaw);

        blink();
        HAL_Delay(10);
    }
    Car_Speed(work.car, 0, 0);
    printf("end yaw:%.2f\r\n", work.YawZ);
}

void turn_right_time(int ms)
{
    Car_Speed(work.car, 0, 0);
    Car_Speed(work.car, 50, 0);
    HAL_Delay(ms);
    Car_Speed(work.car, 0, 0);
}

// void adjust_to_angle(float target_angle)
// {
//     float err = work.zYawZ - target_angle;
//     if (err >= 180)
// }