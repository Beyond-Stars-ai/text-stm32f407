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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_rx;

/* Definitions for DebugTask */
osThreadId_t DebugTaskHandle;
const osThreadAttr_t DebugTask_attributes = {
  .name = "DebugTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LEDTask */
osThreadId_t LEDTaskHandle;
const osThreadAttr_t LEDTask_attributes = {
  .name = "LEDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ReceiveTask */
osThreadId_t ReceiveTaskHandle;
const osThreadAttr_t ReceiveTask_attributes = {
  .name = "ReceiveTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CANTask */
osThreadId_t CANTaskHandle;
const osThreadAttr_t CANTask_attributes = {
  .name = "CANTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ControlQueue */
osMessageQueueId_t ControlQueueHandle;
const osMessageQueueAttr_t ControlQueue_attributes = {
  .name = "ControlQueue"
};
/* USER CODE BEGIN PV */

// 遥控器相关全局变量
uint8_t   dbus_buf[DBUS_BUFLEN];
rc_info_t rc = rc_Init;

// 电机相关全局变量
motor_measure_t motor_chassis[4] = {0};
Motor_Controller_t motor_controller[4] = {0};
int16_t motor_commands[4] = {0};

// CAN 发送相关变量
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CAN1_Init(void);
void StartDebugTask(void *argument);
void StartLEDTask(void *argument);
void StartReceiveTask(void *argument);
void StartCANTask(void *argument);

/* USER CODE BEGIN PFP */

// 遥控器相关函数声明
void uart_receive_handler(UART_HandleTypeDef *huart);
void dbus_uart_init(void);
void rc_callback_handler(rc_info_t *rc, uint8_t *buff);

// 电机控制相关函数声明
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void can_filter_init(void);
void Update_Motor_Measure(uint8_t motor_id, motor_measure_t* measure);
void Motor_PID_Init(void);
void PID_Init(PID_Controller_t* pid, float kp, float ki, float kd, float integral_limit, float output_limit);
float PID_Calculate(PID_Controller_t* pid, float error, float dt);
void Set_Motor_Target_Speed(uint8_t motor_id, float target_speed_rad_s);
void Motor_Speed_Control_Task(float dt);
void Drive_Motor(float vx, float vy, float vz);
float Map_Remote_to_Speed(int16_t input, float max_speed);
void Process_Remote_Control(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 自定义 DMA 接收函数（不使用中断）
static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
  uint32_t tmp1 = 0;
  tmp1 = huart->RxState;
	
	if (tmp1 == HAL_UART_STATE_READY)
	{
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}
 
		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode  = HAL_UART_ERROR_NONE;
 
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);
		
	
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
		
		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}

// 获取当前 DMA 数据计数器
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
  return ((uint16_t)(dma_stream->NDTR));
}

// 遥控器数据解析函数
void rc_callback_handler(rc_info_t *rc, uint8_t *buff)
{
  rc->ch0 = (buff[0] | buff[1] << 8) & 0x07FF;
  rc->ch0 -= 1024;
  rc->ch1 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
  rc->ch1 -= 1024;
  rc->ch2 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
  rc->ch2 -= 1024;
  rc->ch3 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
  rc->ch3 -= 1024;
  rc->roll = (buff[16] | (buff[17] << 8)) & 0x07FF;  
  rc->roll -= 1024;
 
  rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
  rc->sw2 = (buff[5] >> 4) & 0x0003;
	
  if ((abs(rc->ch0) > 660) || \
      (abs(rc->ch1) > 660) || \
      (abs(rc->ch2) > 660) || \
      (abs(rc->ch3) > 660))
  {
    memset(rc, 0, sizeof(rc_info_t));
  }		
}

// UART 空闲中断回调函数
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	
	if (huart == &DBUS_HUART)
	{
		__HAL_DMA_DISABLE(huart->hdmarx);
 
		if ((DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == DBUS_BUFLEN)
		{
			rc_callback_handler(&rc, dbus_buf);	
		}
		__HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}

// UART 接收处理函数
void uart_receive_handler(UART_HandleTypeDef *huart)
{
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && 
			__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		uart_rx_idle_callback(huart);
	}
}

// 遥控器串口初始化
void dbus_uart_init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
	__HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);
	uart_receive_dma_no_it(&DBUS_HUART, dbus_buf, DBUS_MAX_LEN);
}

// RPM 转 rad/s
float RPM_To_RadPerSec(int16_t rpm)
{
    return (float)rpm * 2.0f * 3.1415926535f / 60.0f;
}

// rad/s 转 RPM
int16_t RadPerSec_To_RPM(float rad_per_sec)
{
    return (int16_t)(rad_per_sec * 60.0f / (2.0f * 3.1415926535f));
}

// PID 控制器初始化
void PID_Init(PID_Controller_t* pid, float kp, float ki, float kd, float integral_limit, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
}

// PID 计算函数
float PID_Calculate(PID_Controller_t* pid, float error, float dt)
{
    // 比例项
    float proportional = pid->kp * error;
    
    // 积分项（带限幅）
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    float integral = pid->ki * pid->integral;
    
    // 微分项
    float derivative = pid->kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;
    
    // 计算总输出
    pid->output = proportional + integral + derivative;
    
    // 输出限幅
    if (pid->output > pid->output_limit) {
        pid->output = pid->output_limit;
    } else if (pid->output < -pid->output_limit) {
        pid->output = -pid->output_limit;
    }
    
    return pid->output;
}

// 更新电机测量数据
void Update_Motor_Measure(uint8_t motor_id, motor_measure_t* measure)
{
    if (motor_id < 4) {
        motor_controller[motor_id].measure = *measure;
    }
}

// 初始化所有电机的 PID 控制器
void Motor_PID_Init(void)
{
    // PID参数（需要根据实际电机调试）
    float speed_kp = 32.0f;      // 速度环比例系数
    float speed_ki = 0.8f;      // 速度环积分系数
    float speed_kd = 0.2f;      // 速度环微分系数
    
    for (int i = 0; i < 4; i++) {
        PID_Init(&motor_controller[i].speed_pid, speed_kp, speed_ki, speed_kd, 3000.0f, 3000.0f);
    }
}

// 设置单个电机的目标速度
void Set_Motor_Target_Speed(uint8_t motor_id, float target_speed_rad_s)
{
    if (motor_id < 4) {
        motor_controller[motor_id].target_speed = target_speed_rad_s;
    }
}

// 电机速度控制任务
void Motor_Speed_Control_Task(float dt)
{
    for (int i = 0; i < 4; i++) {
        // 将电机反馈的RPM转换为rad/s
        float actual_speed_rads = RPM_To_RadPerSec(motor_controller[i].measure.speed_rpm);
        
        // 计算速度误差（单位：rad/s）
        float speed_error = motor_controller[i].target_speed - actual_speed_rads;
        
        // PID计算
        float pid_output = PID_Calculate(&motor_controller[i].speed_pid, speed_error, dt);
        
        // 存储控制命令
        motor_commands[i] = (int16_t)pid_output;
    }
}

// 麦克纳姆轮速度解算函数
void Drive_Motor(float vx, float vy, float vz)
{
    float A = WHEELBASE_WIDTH + WHEELBASE_LENGTH;
    
    // 标准麦克纳姆轮运动学逆解算公式
    // 轮子编号：0-右上, 1-左上, 2-左下, 3-右下
    float wheel1_speed = ( vx - vy + vz * A) / WHEEL_RADIUS;  // 右上轮
    float wheel2_speed = ( vx + vy + vz * A) / WHEEL_RADIUS;  // 左上轮
    float wheel3_speed = (-vx + vy + vz * A) / WHEEL_RADIUS;  // 左下轮
    float wheel4_speed = (-vx - vy + vz * A) / WHEEL_RADIUS;  // 右下轮

    Set_Motor_Target_Speed(0, wheel1_speed);  // 右上轮
    Set_Motor_Target_Speed(1, wheel2_speed);  // 左上轮
    Set_Motor_Target_Speed(2, wheel3_speed);  // 左下轮
    Set_Motor_Target_Speed(3, wheel4_speed);  // 右下轮
}

// 遥控器值映射到速度
float Map_Remote_to_Speed(int16_t input, float max_speed)
{
    if (input > 660) input = 660;
    if (input < -660) input = -660;
    
    // 死区内直接返回0
    if (abs(input) < 50) {
        return 0.0f;
    }
    
    // 不在死区内，正常计算速度
    return (input / 660.0f) * max_speed;
}

// 处理遥控器数据并设置底盘速度
void Process_Remote_Control(void)
{
    // 映射遥控器值到速度 (m/s)
    float vx = Map_Remote_to_Speed(rc.ch2, 2.0f);   // 前后方向，最大2m/s
    float vy = Map_Remote_to_Speed(rc.ch3, 2.0f);  // 左右方向，最大2m/s  
    float vz = Map_Remote_to_Speed(rc.ch0, 3.0f);   // 旋转方向，最大3rad/s
    
    // 调用运动学解算
    Drive_Motor(vx, vy, vz);
}

// CAN 总线数据发送函数
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x200;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;
    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, 
        chassis_can_send_data, &send_mail_box);
}

// CAN 滤波器初始化
void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0007;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  dbus_uart_init();
  HAL_CAN_Start(&hcan1);
  Motor_PID_Init();
  can_filter_init();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of ControlQueue */
  ControlQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &ControlQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of DebugTask */
  DebugTaskHandle = osThreadNew(StartDebugTask, NULL, &DebugTask_attributes);

  /* creation of LEDTask */
  LEDTaskHandle = osThreadNew(StartLEDTask, NULL, &LEDTask_attributes);

  /* creation of ReceiveTask */
  ReceiveTaskHandle = osThreadNew(StartReceiveTask, NULL, &ReceiveTask_attributes);

  /* creation of CANTask */
  CANTaskHandle = osThreadNew(StartCANTask, NULL, &CANTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 100000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, LED_R_Pin|LED_G_Pin|LED_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_R_Pin LED_G_Pin LED_B_Pin */
  GPIO_InitStruct.Pin = LED_R_Pin|LED_G_Pin|LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int fd, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDebugTask */
/**
  * @brief  Function implementing the DebugTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDebugTask */
void StartDebugTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  printf("Debug Task: Remote Control Monitor\r\n");
  printf("CH0(旋转) CH1 CH2(前后) CH3(左右) SW1 SW2\r\n");
  /* Infinite loop */
  for(;;)
  {
    // 打印遥控器通道值，用于调试信号传输问题
    printf("RC: %4d %4d %4d %4d %d %d\r\n", 
           rc.ch0, rc.ch1, rc.ch2, rc.ch3, rc.sw1, rc.sw2);
    
    // 检查遥控器数据是否有效（非零值表示有信号）
    if (rc.ch0 != 0 || rc.ch1 != 0 || rc.ch2 != 0 || rc.ch3 != 0)
    {
      printf("RC Signal: ACTIVE\r\n");
    }
    else
    {
      printf("RC Signal: NO DATA\r\n");
    }
    
    // 打印原始数据缓冲区的前几个字节，用于调试原始信号
    printf("Raw: %02X %02X %02X %02X %02X %02X\r\n",
           dbus_buf[0], dbus_buf[1], dbus_buf[2], dbus_buf[3], dbus_buf[4], dbus_buf[5]);
    
    osDelay(200);  // 200ms 刷新频率，便于观察信号变化
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLEDTask */
/**
* @brief Function implementing the LEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLEDTask */
void StartLEDTask(void *argument)
{
  /* USER CODE BEGIN StartLEDTask */
  /* Infinite loop */
  for(;;)
  {
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

/* USER CODE BEGIN Header_StartReceiveTask */
/**
* @brief Function implementing the ReceiveTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReceiveTask */
void StartReceiveTask(void *argument)
{
  /* USER CODE BEGIN StartReceiveTask */
  printf("ReceiveTask is running...\r\n");
  
  // 初始化遥控器串口接收
  dbus_uart_init();
  
  /* Infinite loop */
  for(;;)
  {
    // 这里可以添加遥控器数据处理逻辑
    // 例如：检查遥控器数据是否更新，处理异常等
    
    osDelay(10);  // 10ms 周期检查
  }
  /* USER CODE END StartReceiveTask */
}

/* USER CODE BEGIN Header_StartCANTask */
/**
* @brief Function implementing the CANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCANTask */
void StartCANTask(void *argument)
{
  /* USER CODE BEGIN StartCANTask */
  printf("CANTask is running...\r\n");
  
  uint32_t last_control_time = 0;
  uint32_t dt = 0;
  
  /* Infinite loop */
  for(;;)
  {
    uint32_t current_time = HAL_GetTick();
   
    // 控制任务：固定频率执行（50Hz，20ms）
    if (current_time - last_control_time >= 20)
    {
      dt = (current_time - last_control_time) * 0.001f;
      
      // 处理遥控器数据并计算目标速度
      Process_Remote_Control();
      
      // 执行电机速度控制
      Motor_Speed_Control_Task(dt);
      
      // 发送 CAN 命令到电机
      CAN_cmd_chassis(motor_commands[0], motor_commands[1], motor_commands[2], motor_commands[3]);
      
      last_control_time = current_time;
    }
    
    osDelay(1);  // 1ms 基础延迟
  }
  /* USER CODE END StartCANTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
