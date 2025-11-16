/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t key_exit_flag = 0;
uint8_t key_rising_falling_flag;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for LEDTask */
osThreadId_t LEDTaskHandle;
const osThreadAttr_t LEDTask_attributes = {
  .name = "LEDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BuzzerTask */
osThreadId_t BuzzerTaskHandle;
const osThreadAttr_t BuzzerTask_attributes = {
  .name = "BuzzerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for KEYTask */
osThreadId_t KEYTaskHandle;
const osThreadAttr_t KEYTask_attributes = {
  .name = "KEYTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PrintTask */
osThreadId_t PrintTaskHandle;
const osThreadAttr_t PrintTask_attributes = {
  .name = "PrintTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartLEDTask(void *argument);
void StartBuzzerTask(void *argument);
void StartKEYTask(void *argument);
void StartPrintTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LEDTask */
  LEDTaskHandle = osThreadNew(StartLEDTask, NULL, &LEDTask_attributes);

  /* creation of BuzzerTask */
  BuzzerTaskHandle = osThreadNew(StartBuzzerTask, NULL, &BuzzerTask_attributes);

  /* creation of KEYTask */
  KEYTaskHandle = osThreadNew(StartKEYTask, NULL, &KEYTask_attributes);

  /* creation of PrintTask */
  PrintTaskHandle = osThreadNew(StartPrintTask, NULL, &PrintTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartLEDTask */
/**
 * @brief  Function implementing the LEDTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLEDTask */
void StartLEDTask(void *argument)
{
  /* USER CODE BEGIN StartLEDTask */
  /* Infinite loop */
  for (;;) {
    HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
    osDelay(200);
    HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
    osDelay(200);
    HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
    osDelay(200);
    HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
    osDelay(200);
    HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
    osDelay(200);
    HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
    osDelay(200);
  }
  /* USER CODE END StartLEDTask */
}

/* USER CODE BEGIN Header_StartBuzzerTask */
/**
 * @brief Function implementing the BuzzerTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBuzzerTask */
void StartBuzzerTask(void *argument)
{
  /* USER CODE BEGIN StartBuzzerTask */
  int MAX_BUZZER_PWM = 20000;
  int MIN_BUZZER_PWM = 10000;
  int MAX_PSC = 1000;
  int pwm = MIN_BUZZER_PWM;
  int psc = 0;
  /* Infinite loop */
  for (;;) {
    if (pwm > MAX_BUZZER_PWM) {
      pwm = MIN_BUZZER_PWM;
    }
    if (psc > MAX_PSC) {
      psc = 0;
    }

    __HAL_TIM_SET_PRESCALER(&htim4, psc);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm);

    osDelay(25);
  }
  /* USER CODE END StartBuzzerTask */
}

/* USER CODE BEGIN Header_StartKEYTask */
/**
 * @brief Function implementing the KEYTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartKEYTask */
void StartKEYTask(void *argument)
{
  /* USER CODE BEGIN StartKEYTask */
  /* Infinite loop */
  for (;;) {
    if (key_exit_flag == 1) {
      key_exit_flag = 2;
      if (key_rising_falling_flag == GPIO_PIN_RESET) {
        // debouce
        osDelay(20);
        if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET) {
          HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
          key_exit_flag = 0;
        } else {
          key_exit_flag = 0;
        }
      } else if (key_rising_falling_flag == GPIO_PIN_SET) {
        // debouce
        osDelay(20);
        if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET) {
          HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
          key_exit_flag = 0;
        } else {
          key_exit_flag = 0;
        }
      }
    }
    osDelay(10);
  }
  /* USER CODE END StartKEYTask */
}

/* USER CODE BEGIN Header_StartPrintTask */
/**
* @brief Function implementing the PrintTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPrintTask */
void StartPrintTask(void *argument)
{
  /* USER CODE BEGIN StartPrintTask */
  int count = 0;
  /* Infinite loop */
  for(;;)
  {
    printf("Hello World!\r\n"); // 循环输出字符串
    printf("Count: %d\r\n", count++); // 循环输出变量
    osDelay(1000);
  }
  /* USER CODE END StartPrintTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == KEY_Pin) {
    if (key_exit_flag == 0) {
      key_exit_flag = 1;
      key_rising_falling_flag = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
    }
  }
}

/* USER CODE END Application */

