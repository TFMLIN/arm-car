#include "trace.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"

uint8_t Trace_ReadRegister()
{
    uint8_t data;
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&hi2c1, DEVICE_ADDR | 0x01, &data, 1, 100);
    if (status != HAL_OK) {
        // 处理错误
        // 可以添加错误处理代码，例如重试或记录错误
        printf("I2C Read Error: %d\r\n", status);
        return 0xFF; // 返回一个错误值
    }
    return data;
}