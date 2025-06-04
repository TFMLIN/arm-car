/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "wit_c_sdk.h"
#include "witi2c.h"
#include "motor.h"
#include "trace.h"
#include "pca9685.h"
#include "servo.h"
#include "work.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ACC_UPDATE   0x01
#define GYRO_UPDATE  0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE   0x08
#define READ_UPDATE  0x80

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float RollX = 0, PitchY = 0, YawZ;
float zRollx, zPitchY, zYawZ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t RxData1, RxData2;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM6_Init();
    MX_TIM4_Init();
    MX_USART3_UART_Init();
    /* USER CODE BEGIN 2 */
    printf("begin\r\n");
    HAL_UART_Receive_IT(&huart1, &RxData1, 1);
    HAL_UART_Receive_IT(&huart2, &RxData2, 1); // 启动UART接收中断
    HAL_Delay(100);
    zRollx = RollX, zPitchY = PitchY, zYawZ = YawZ;
    if (zYawZ == 360) {
        zYawZ = 0; // 初始化YawZ为0
    }

    // 启用调试输出
    // PCA9685_SetDebug(1);

    // 初始化PCA9685
    PCA9685_Init(&hi2c1, 1);

    // 设置PWM频率为50Hz（舵机常用频率）
    PCA9685_SetPWMFreq(&hi2c1, 50.0f);
    int mid = 1570;
    PCA9685_SetServoPulse(&hi2c1, 0, mid); // 设置第一个舵机到中位1.5ms
    Car_Init(&car);
    Car_Speed_Percentage(&car, 0, 0);

    for (int i = 0; i < 6; i++) {
        HAL_GPIO_WritePin(GPIOB, LED3_Pin, (i & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_Delay(100);
    }

    // int turn_pulse = 2180;
    // PCA9685_SetServoPulse(&hi2c1, 0, mid + 500);
    // PCA9685_SetServoPulse(&hi2c1, 0, turn_pulse);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    int i = 0, led = 0;
    // int mode = -1;
    HAL_Delay(1000);                 // 等待1秒钟，确保系统稳定
    WorkInit(&car, &pca9685_handle); // 初始化工作状态
    mode0();
    // mode1();         // 进入模式1，开始直行
    HAL_Delay(1000); // 等待1秒钟，确保系统稳定
    mode2();
    while (1) {
        // uint8_t trace_state = Trace_ReadRegister();
        // printf("in main Trace State: %02X\r\n", trace_state);
        // if (YawZ > 180) {
        //     YawZ -= 360;
        // }
        // int servo_pulse = mid + (car.speed > 0 ? 1 : -1) * (int)(YawZ * 30); // 将YawZ转换为舵机脉冲宽度
        // if (servo_pulse < 1000) servo_pulse = 1000;                          // 限制最小脉冲宽度
        // if (servo_pulse > 2000) servo_pulse = 2000;                          // 限制最大脉冲宽度
        // printf("YawZ: %.2f, Servo Pulse: %d\r\n", YawZ, servo_pulse);
        // PCA9685_SetServoPulse(&hi2c1, 0, servo_pulse); // 设置舵机脉冲宽度
        i++;
        if (i == 10) {
            i   = 0;
            led = !led;
            HAL_GPIO_WritePin(GPIOB, LED3_Pin, led ? GPIO_PIN_SET : GPIO_PIN_RESET); // 切换LED状态
        }
        HAL_Delay(10);
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 8;
    RCC_OscInitStruct.PLL.PLLN            = 168;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */
    if (htim->Instance == TIM6) {
        Motor_Speed_Control(&car);
    }
    /* USER CODE END Callback 0 */
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

void jy61p_ReceiveData(uint8_t data)
{
    static uint8_t index = 0;
    static uint8_t buffer[15];
    if (data == 0x55) { // 帧头
        index           = 0;
        buffer[index++] = data;
    } else if (index > 0 && index < 11) {
        // printf("i:%d d:%d\r\n", index, data);
        buffer[index++] = data;
        if (index == 11) {
            // 解析数据
            if (buffer[1] != 0x53) return;
            uint8_t sum = 0x55 + 0x53 + buffer[2] + buffer[3] + buffer[4] + buffer[5] + buffer[6] + buffer[7] + buffer[8] + buffer[9];
            if (sum != buffer[10]) return;
            // printf("data received: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], buffer[8], buffer[9], buffer[10]);
            work.RollX  = (float)(((buffer[3] << 8) | buffer[2]) / 32768.0 * 180) - work.zRollx;
            work.PitchY = (float)(((buffer[5] << 8) | buffer[4]) / 32768.0 * 180) - work.zPitchY;
            work.YawZ   = (float)(((buffer[7] << 8) | buffer[6]) / 32768.0 * 180) - work.zYawZ;
            if (work.YawZ < 0) {
                work.YawZ += 360; // 确保YawZ在0到360度之间
            }
            // 处理`角度数据
            // printf("%.2f,%.2f,%.2f\r\n", RollX, PitchY, YawZ);
        }
    }
}

void control_ReceiveData(uint8_t data)
{
    static uint8_t index = 0;
    static uint8_t buffer[15];
    if (data == '@') { // 帧头
        index           = 0;
        buffer[index++] = data;
    } else if (data == '\n') {
        int16_t speed[2];
        speed[0] = ((buffer[1] & 0x7f) << 8) | buffer[2];
        speed[1] = ((buffer[3] & 0x7f) << 8) | buffer[4];
        if (buffer[1] & 0x80) speed[0] = -speed[0]; // 处理方向
        if (buffer[3] & 0x80) speed[1] = -speed[1]; // 处理方向
        if (index == 5) {
            // 解析数据
            // printf("data received: %02X %02X %02X %02X %02X\r\n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
            printf("Speed:%d,%d\r\n", speed[0], speed[1]);
            Car_Speed(&car, speed[0], speed[1]);
        }
    } else {
        buffer[index++] = data;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        control_ReceiveData(RxData1);              // 调用数据包处理函数
        HAL_UART_Receive_IT(&huart1, &RxData1, 1); // 继续接收下一个字节
    } else if (huart->Instance == USART2) {
        // printf("interrupt received: %02X\r\n", RxData);
        jy61p_ReceiveData(RxData2);                // 调用数据包处理函数
        HAL_UART_Receive_IT(&huart2, &RxData2, 1); // 继续接收下一个字节
    }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
