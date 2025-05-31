#include "pca9685.h"
#include "stdio.h"
#include "math.h"

// Internal I2C handle for the module
static I2C_HandleTypeDef *pca9685_hi2c = NULL;
static uint8_t pca9685_address         = 0x40 << 1;
static uint8_t debug_enabled           = 0;

// Private function prototypes
static HAL_StatusTypeDef PCA9685_Write(uint8_t reg, uint8_t value);
static uint8_t PCA9685_Read(uint8_t reg);

// Private function implementations
static HAL_StatusTypeDef PCA9685_Write(uint8_t reg, uint8_t value)
{
    uint8_t data[2]          = {reg, value};
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(pca9685_hi2c, pca9685_address, data, 2, 100);

    if (debug_enabled) {
        printf("I2C: Write 0x%02X to register 0x%02X\r\n", value, reg);
    }

    if (status != HAL_OK) {
        printf("PCA9685 Write Error: %d\r\n", status);
    }

    return status;
}

static uint8_t PCA9685_Read(uint8_t reg)
{
    uint8_t result = 0;
    HAL_StatusTypeDef status;

    // First write the register address
    status = HAL_I2C_Master_Transmit(pca9685_hi2c, pca9685_address, &reg, 1, 100);
    if (status != HAL_OK) {
        printf("PCA9685 Read Address Error: %d\r\n", status);
        return 0;
    }

    // Then read the data
    status = HAL_I2C_Master_Receive(pca9685_hi2c, pca9685_address, &result, 1, 100);
    if (status != HAL_OK) {
        printf("PCA9685 Read Data Error: %d\r\n", status);
        return 0;
    }

    if (debug_enabled) {
        printf("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X\r\n",
               pca9685_address >> 1, result, reg);
    }

    return result;
}

void PCA9685_Init(I2C_HandleTypeDef *hi2c)
{
    printf("PCA9685 Init\r\n");

    // Store the I2C handle for later use
    pca9685_hi2c = hi2c;

    if (debug_enabled) {
        printf("Reseting PCA9685\r\n");
    }

    // Reset the device to normal mode
    PCA9685_Write(PCA9685_MODE1, 0x00);
}

void PCA9685_SetDebug(uint8_t enable)
{
    debug_enabled = enable;
}

void PCA9685_SetPWMFreq(I2C_HandleTypeDef *hi2c, float freq)
{
    // Store the I2C handle if not already stored
    if (pca9685_hi2c == NULL) {
        pca9685_hi2c = hi2c;
    }

    // Calculate prescale value
    float prescaleval = 25000000.0f; // 25MHz internal oscillator
    prescaleval /= 4096.0f;          // 12-bit resolution
    prescaleval /= freq;
    prescaleval -= 1.0f;

    if (debug_enabled) {
        printf("Setting PWM frequency to %.1f Hz\r\n", freq);
        printf("Estimated pre-scale: %.1f\r\n", prescaleval);
    }

    uint8_t prescale = (uint8_t)(prescaleval + 0.5f); // Round to nearest integer

    if (debug_enabled) {
        printf("Final pre-scale: %d\r\n", prescale);
    }

    // Read current mode1 register
    uint8_t oldmode = PCA9685_Read(PCA9685_MODE1);
    uint8_t newmode = (oldmode & 0x7F) | 0x10; // Set sleep bit

    // Go to sleep to change prescale
    PCA9685_Write(PCA9685_MODE1, newmode);

    // Set prescale
    PCA9685_Write(PCA9685_PRESCALE, prescale);

    // Restore old mode
    PCA9685_Write(PCA9685_MODE1, oldmode);

    // Wait for oscillator to stabilize
    HAL_Delay(5);

    // Set restart bit
    PCA9685_Write(PCA9685_MODE1, oldmode | 0x80);
}

void PCA9685_SetPWM(I2C_HandleTypeDef *hi2c, uint8_t channel, uint16_t on, uint16_t off)
{
    // Store the I2C handle if not already stored
    if (pca9685_hi2c == NULL) {
        pca9685_hi2c = hi2c;
    }

    if (debug_enabled) {
        printf("channel: %d  LED_ON: %d LED_OFF: %d\r\n", channel, on, off);
    }

    // Write ON registers
    PCA9685_Write(PCA9685_LED0_ON_L + 4 * channel, on & 0xFF);
    PCA9685_Write(PCA9685_LED0_ON_H + 4 * channel, on >> 8);

    // Write OFF registers
    PCA9685_Write(PCA9685_LED0_OFF_L + 4 * channel, off & 0xFF);
    PCA9685_Write(PCA9685_LED0_OFF_H + 4 * channel, off >> 8);
}

void PCA9685_SetServoPulse(I2C_HandleTypeDef *hi2c, uint8_t channel, uint16_t pulse)
{
    // Store the I2C handle if not already stored
    if (pca9685_hi2c == NULL) {
        pca9685_hi2c = hi2c;
    }

    // Calculate pulse value for 50Hz PWM (20ms period)
    // pulse is in microseconds, convert to 4096 scale
    uint16_t pulse_value = (uint16_t)((float)pulse * 4096.0f / 20000.0f);

    if (debug_enabled) {
        printf("SetServoPulse Channel:%d Pulse:%dus Value:%d\r\n", channel, pulse, pulse_value);
    }

    PCA9685_SetPWM(hi2c, channel, 0, pulse_value);
}

void PCA9685_SetServoAngle(I2C_HandleTypeDef *hi2c, uint8_t channel, float angle)
{
    // Ensure angle is within 0-180 degrees
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    // Convert angle to pulse width (500us to 2500us)
    uint16_t pulse = (uint16_t)(500 + (angle / 180.0f) * 2000);

    if (debug_enabled) {
        printf("SetServoAngle Channel:%d Angle:%.2f Pulse:%dus\r\n", channel, angle, pulse);
    }

    PCA9685_SetServoPulse(hi2c, channel, pulse);
}