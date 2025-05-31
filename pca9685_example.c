// PCA9685 使用示例
// 这个文件展示了如何使用新的PCA9685驱动程序
// 类似于Python版本的功能

#include "pca9685.h"
#include "main.h"

void PCA9685_Example(I2C_HandleTypeDef *hi2c)
{
    // 启用调试输出
    PCA9685_SetDebug(1);

    // 初始化PCA9685
    PCA9685_Init(hi2c);

    // 设置PWM频率为50Hz（舵机常用频率）
    PCA9685_SetPWMFreq(hi2c, 50.0f);

    while (1) {
        // 舵机控制示例：从500μs到2500μs脉宽变化
        for (uint16_t i = 500; i < 2500; i += 10) {
            PCA9685_SetServoPulse(hi2c, 0, i);
            HAL_Delay(20); // 延时20ms
        }

        for (uint16_t i = 2500; i > 500; i -= 10) {
            PCA9685_SetServoPulse(hi2c, 0, i);
            HAL_Delay(20); // 延时20ms
        }
    }
}

// 其他使用示例：
void PCA9685_MultiChannel_Example(I2C_HandleTypeDef *hi2c)
{
    PCA9685_Init(hi2c);
    PCA9685_SetPWMFreq(hi2c, 50.0f);

    // 控制多个舵机
    PCA9685_SetServoPulse(hi2c, 0, 1500); // 通道0，中位1.5ms
    PCA9685_SetServoPulse(hi2c, 1, 1000); // 通道1，最小1ms
    PCA9685_SetServoPulse(hi2c, 2, 2000); // 通道2，最大2ms

    // 直接设置PWM值
    PCA9685_SetPWM(hi2c, 3, 0, 2048); // 通道3，50%占空比
}
