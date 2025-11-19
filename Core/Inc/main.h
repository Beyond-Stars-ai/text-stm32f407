/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOH
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOH
#define LED_B_Pin GPIO_PIN_10
#define LED_B_GPIO_Port GPIOH

/* USER CODE BEGIN Private defines */

// DBUS 遥控器相关定义
#define DBUS_MAX_LEN     (50)
#define DBUS_BUFLEN      (18)
#define DBUS_HUART       huart3

// 底盘参数（根据实际机械结构调整）
#define WHEEL_RADIUS          0.175f    // 轮子半径 (米)
#define WHEELBASE_WIDTH       0.135f    // 轮距宽度/2 (米)
#define WHEELBASE_LENGTH      0.2f      // 轮距长度/2 (米)

// 遥控器初始化值
#define rc_Init   \
{                 \
    0,            \
    0,            \
    0,            \
    0,            \
    0,            \
    0,            \
    0,            \
}

// 遥控器数据结构
typedef struct
{
  int16_t ch0;
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t roll;
  uint8_t sw1;
  uint8_t sw2;
} rc_info_t;

// 电机测量数据结构
typedef struct
{
  uint16_t ecd;           // 编码器值 -16384--+16384
  int16_t speed_rpm;      // 转速 转每分钟
  int16_t given_current;  // 输出电流值
  uint8_t temperate;      // 电机温度
  int16_t last_ecd;       // 上次编码器值
} motor_measure_t;

// PID 控制器结构体
typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数  
    float kd;           // 微分系数
    float integral;     // 积分项
    float prev_error;   // 上一次误差
    float output;       // 输出值
    float integral_limit; // 积分限幅
    float output_limit;   // 输出限幅
} PID_Controller_t;

// 电机控制结构体
typedef struct {
    motor_measure_t measure;    // 电机测量值（速度单位：RPM）
    PID_Controller_t speed_pid; // 速度环PID
    float target_speed;         // 目标速度 (rad/s)
} Motor_Controller_t;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
