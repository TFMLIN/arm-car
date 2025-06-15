# 基于stm32f407的自动行驶小车

## 整体结构

### 器件选型

#### stm32f407vgt6
#### tb6612+520编码电机
#### 亚博智能8路灰度传感器
带有8根直传信号线+1个i2c接口+1个uart接口

可以通过i2c接口或者uart接口有效减少开发板的接口占用

#### pca9685+25kg数字舵机

带有一个i2c接口和16路的舵机控制接口

#### jy61p陀螺仪

6轴陀螺仪，零飘小，精度高达±0.5°

### 连线

stm32通过杜邦线连接tb6612驱动板，驱动板连接后面两个520编码电机

stm32通过i2c1接口连接一个i2c bus，i2c bus下挂2个i2c设备，其中一个是8路灰度巡线，另一个是pca9685，可以实现io接口的复用

stm32通过uart1连接串口，通过uart2连接jy61p

pca9685通过pwm线和电源线连接舵机

### 控制
stm32通过2个pwm接口、4个普通io接口、4个timer接口连接tb6612驱动板，实现控制电机的正反转，控制电机的占空比控制电机转速，通过时钟线来接收编码电机的脉冲，从而得知电机的实际转速，再根据pid算法来调整占空比从而达到精准的速度控制。

8路灰度通过i2c传回传感器的数据给stm32，当巡线时，根据灰度的01值来判断目前车头和循迹线的相对位置。

当循迹线处于偏左位置，则通过i2c想pca9685发送较长高电平的脉冲给舵机，控制前轮左转，反之发送较短电平的脉冲给舵机，控制前轮右转。

当小车走直线时，由于车身结构原因，小车可能会有轻微角度的偏移，所以我们采用陀螺仪来校准方向，当角度偏离过大时能够及时纠正。

## 模块介绍
```
│   gpio.c  // gpio配置
│   i2c.c   // i2c配置
│   main.c  // 主程序
│   motor.c // 电机控制
│   pca9685.c   // 舵机控制
│   pid.c   // pid算法的实现
│   servo.c // pwm控制舵机
│   stm32f4xx_hal_msp.c
│   stm32f4xx_it.c
│   syscalls.c
│   sysmem.c
│   system_stm32f4xx.c
│   tim.c   // 定时器配置
│   trace.c // 循迹模块
│   usart.c // 串口配置
│   witi2c.c    // 陀螺仪
│   work.c  // 题目逻辑的实现
```

### 主程序
`void jy61p_ReceiveData(uint8_t data)`函数用于处理陀螺仪发来的数据
`void control_ReceiveData(uint8_t data)`函数用来处理串口调试助手发来的电机控制语句，用于调试
`void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)`用于接收中断回调
### 电机控制
`void Car_Init(CarConfig *car)`用于初始化所有电机
`void Car_Speed_PWM(CarConfig *car, int s1, int s2)`用于设置各个电机的PWM
`void Car_Speed_Percentage(CarConfig *car, int8_t p1, int8_t p2)`用于设置电机转速和最大转速的百分比
`void Car_Speed(CarConfig *car, int s1, int s2)`用于设置各个电机转速，对应于编码器每个计时单位的脉冲数
### pid控制
`void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float setpoint, float max_control)`用于初始化pid参数
`void PID_Reset(PID_Controller *pid)`用于重置pid的累计误差
`float PID_Compute(PID_Controller *pid, float measured_value, float dt)`计算得到控制量
### 舵机控制
### 循迹模块
### 陀螺仪