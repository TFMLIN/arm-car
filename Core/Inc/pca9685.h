#ifndef __PCA9685_H
#define __PCA9685_H

#include "stm32f4xx_hal.h"
#include "i2c.h"

#define PCA9685_ADDRESS 0x40 << 1 // 7-bit address left shifted for HAL I2C

// Register definitions based on Python version
#define PCA9685_SUBADR1       0x02
#define PCA9685_SUBADR2       0x03
#define PCA9685_SUBADR3       0x04
#define PCA9685_MODE1         0x00
#define PCA9685_MODE2         0x01
#define PCA9685_PRESCALE      0xFE // Prescale register for setting PWM frequency
#define PCA9685_LED0_ON_L     0x06 // LED0 ON low byte
#define PCA9685_LED0_ON_H     0x07 // LED0 ON high byte
#define PCA9685_LED0_OFF_L    0x08 // LED0 OFF low byte
#define PCA9685_LED0_OFF_H    0x09 // LED0 OFF high byte
#define PCA9685_ALL_LED_ON_L  0xFA // All LEDs ON low byte
#define PCA9685_ALL_LED_ON_H  0xFB // All LEDs ON high byte
#define PCA9685_ALL_LED_OFF_L 0xFC // All LEDs OFF low byte
#define PCA9685_ALL_LED_OFF_H 0xFD // All LEDs OFF high byte
#define PCA9685_RESTART       0x80 // Restart bit for mode register
#define PCA9685_SLEEP         0x10 // Sleep bit for mode register

// Function declarations
void PCA9685_Init(I2C_HandleTypeDef *hi2c);
void PCA9685_SetDebug(uint8_t enable);
void PCA9685_SetPWMFreq(I2C_HandleTypeDef *hi2c, float freq);
void PCA9685_SetPWM(I2C_HandleTypeDef *hi2c, uint8_t channel, uint16_t on, uint16_t off);
void PCA9685_SetServoPulse(I2C_HandleTypeDef *hi2c, uint8_t channel, uint16_t pulse);
void PCA9685_SetServoAngle(I2C_HandleTypeDef *hi2c, uint8_t channel, float angle);

#endif // !__PCA9685_H