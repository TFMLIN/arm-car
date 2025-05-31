# PCA9685 C语言驱动程序

这是一个基于Python版本翻译的PCA9685 16通道PWM驱动器的C语言驱动程序，适用于STM32平台。

## 功能特性

- 支持16通道PWM输出
- 可配置PWM频率（1-1526 Hz）
- 舵机专用脉宽控制函数
- 调试模式支持
- 与STM32 HAL库兼容

## 主要函数

### 初始化和配置
```c
void PCA9685_Init(I2C_HandleTypeDef *hi2c);
void PCA9685_SetDebug(uint8_t enable);
void PCA9685_SetPWMFreq(I2C_HandleTypeDef *hi2c, float freq);
```

### PWM控制
```c
void PCA9685_SetPWM(I2C_HandleTypeDef *hi2c, uint8_t channel, uint16_t on, uint16_t off);
void PCA9685_SetServoPulse(I2C_HandleTypeDef *hi2c, uint8_t channel, uint16_t pulse);
```

## 使用示例

### 基本初始化
```c
I2C_HandleTypeDef hi2c1;  // 假设你已经配置了I2C

// 初始化PCA9685
PCA9685_Init(&hi2c1);

// 设置PWM频率为50Hz（舵机常用频率）
PCA9685_SetPWMFreq(&hi2c1, 50.0f);
```

### 舵机控制
```c
// 设置舵机到中位（1.5ms脉宽）
PCA9685_SetServoPulse(&hi2c1, 0, 1500);

// 设置舵机到最小位置（1ms脉宽）
PCA9685_SetServoPulse(&hi2c1, 0, 1000);

// 设置舵机到最大位置（2ms脉宽）
PCA9685_SetServoPulse(&hi2c1, 0, 2000);
```

### 直接PWM控制
```c
// 通道0，50%占空比
PCA9685_SetPWM(&hi2c1, 0, 0, 2048);

// 通道1，25%占空比
PCA9685_SetPWM(&hi2c1, 1, 0, 1024);
```

### 调试模式
```c
// 启用调试输出
PCA9685_SetDebug(1);

// 现在所有I2C操作都会输出调试信息
PCA9685_SetServoPulse(&hi2c1, 0, 1500);
```

## 与Python版本的对应关系

| Python方法 | C函数 | 说明 |
|------------|-------|------|
| `__init__()` | `PCA9685_Init()` | 初始化设备 |
| `write()` | `PCA9685_Write()` (内部函数) | 写寄存器 |
| `read()` | `PCA9685_Read()` (内部函数) | 读寄存器 |
| `setPWMFreq()` | `PCA9685_SetPWMFreq()` | 设置PWM频率 |
| `setPWM()` | `PCA9685_SetPWM()` | 设置PWM值 |
| `setServoPulse()` | `PCA9685_SetServoPulse()` | 设置舵机脉宽 |
| `debug` 属性 | `PCA9685_SetDebug()` | 调试模式控制 |

## 寄存器定义

所有寄存器定义都在 `pca9685.h` 中，与Python版本保持一致：

- `PCA9685_MODE1`: 模式寄存器1
- `PCA9685_PRESCALE`: 预分频寄存器
- `PCA9685_LED0_ON_L/H`: LED0开启时间寄存器
- `PCA9685_LED0_OFF_L/H`: LED0关闭时间寄存器
- 等等...

## 注意事项

1. 确保I2C已正确配置并初始化
2. PCA9685的默认I2C地址是0x40
3. 舵机控制需要50Hz的PWM频率
4. 脉宽范围通常是500-2500微秒
5. 调试输出需要配置printf重定向到UART

## 改进点

相比原版本，新版本具有以下改进：

1. 添加了调试模式控制
2. 更准确的PWM频率计算
3. 完整的错误处理
4. 更好的代码结构和注释
5. 与Python版本功能完全对应
