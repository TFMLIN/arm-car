#include "witi2c.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include "i2c.h"
//陀螺仪
int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[length + 1];
    buffer[0] = reg; // 第一个字节是寄存器地址
    for (uint32_t i = 0; i < length; i++) {
        buffer[i + 1] = data[i]; // 后续字节是数据
    }

    // 发送数据
    status = HAL_I2C_Master_Transmit(&hi2c1, dev << 1, buffer, length + 1, I2C_TIMEOUT);
    if (status != HAL_OK) {
        return -1; // 发送数据失败
    }

    return 0; // 成功
}

int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length)
{
    HAL_StatusTypeDef status;
    uint8_t regAddr = reg;

    // 发送寄存器地址
    status = HAL_I2C_Master_Transmit(&hi2c1, dev << 1, &regAddr, 1, I2C_TIMEOUT);
    if (status != HAL_OK) {
        return -1; // 发送寄存器地址失败
    }

    // 接收数据
    status = HAL_I2C_Master_Receive(&hi2c1, dev << 1, data, length, I2C_TIMEOUT);
    if (status != HAL_OK) {
        return -1; // 接收数据失败
    }

    return 0; // 成功
}