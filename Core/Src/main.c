/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "APP.h"

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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* Definitions for SoilSensorTask */
osThreadId_t SoilSensorTaskHandle;
const osThreadAttr_t SoilSensorTask_attributes = {
  .name = "SoilSensorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UartPrintTask */
osThreadId_t UartPrintTaskHandle;
const osThreadAttr_t UartPrintTask_attributes = {
  .name = "UartPrintTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PumpControlTask */
osThreadId_t PumpControlTaskHandle;
const osThreadAttr_t PumpControlTask_attributes = {
  .name = "PumpControlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ButtonTask */
osThreadId_t ButtonTaskHandle;
const osThreadAttr_t ButtonTask_attributes = {
  .name = "ButtonTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for WaterTankTask */
osThreadId_t WaterTankTaskHandle;
const osThreadAttr_t WaterTankTask_attributes = {
  .name = "WaterTankTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SystemMonitorTa */
osThreadId_t SystemMonitorTaHandle;
const osThreadAttr_t SystemMonitorTa_attributes = {
  .name = "SystemMonitorTa",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SystemStatusTas */
osThreadId_t SystemStatusTasHandle;
const osThreadAttr_t SystemStatusTas_attributes = {
  .name = "SystemStatusTas",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for OledDisplayTask */
osThreadId_t OledDisplayTaskHandle;
const osThreadAttr_t OledDisplayTask_attributes = {
  .name = "OledDisplayTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SoilAdcQueue */
osMessageQueueId_t SoilAdcQueueHandle;
const osMessageQueueAttr_t SoilAdcQueue_attributes = {
  .name = "SoilAdcQueue"
};
/* Definitions for SystemStatusQueue */
osMessageQueueId_t SystemStatusQueueHandle;
const osMessageQueueAttr_t SystemStatusQueue_attributes = {
  .name = "SystemStatusQueue"
};
/* Definitions for OledStatusQueue */
osMessageQueueId_t OledStatusQueueHandle;
const osMessageQueueAttr_t OledStatusQueue_attributes = {
  .name = "OledStatusQueue"
};
/* Definitions for UartMutex */
osMutexId_t UartMutexHandle;
const osMutexAttr_t UartMutex_attributes = {
  .name = "UartMutex"
};
/* Definitions for I2cMutex */
osMutexId_t I2cMutexHandle;
const osMutexAttr_t I2cMutex_attributes = {
  .name = "I2cMutex"
};
/* Definitions for SoilEvent */
osEventFlagsId_t SoilEventHandle;
const osEventFlagsAttr_t SoilEvent_attributes = {
  .name = "SoilEvent"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
void StartSoilSensorTask(void *argument);
void StartUartPrintTask(void *argument);
void StartPumpControlTask(void *argument);
void StartButtonTask(void *argument);
void StartWaterTankTask(void *argument);
void StartSystemMonitorTask(void *argument);
void StartSystemStatusTask(void *argument);
void StartOledDisplayTask(void *argument);

/* USER CODE BEGIN PFP */

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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  APP_Init();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of UartMutex */
  UartMutexHandle = osMutexNew(&UartMutex_attributes);

  /* creation of I2cMutex */
  I2cMutexHandle = osMutexNew(&I2cMutex_attributes);

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
  /* creation of SoilAdcQueue */
  SoilAdcQueueHandle = osMessageQueueNew (5, sizeof(SoilData_t), &SoilAdcQueue_attributes);

  /* creation of SystemStatusQueue */
  SystemStatusQueueHandle = osMessageQueueNew (5, sizeof(SystemStatus_t), &SystemStatusQueue_attributes);

  /* creation of OledStatusQueue */
  OledStatusQueueHandle = osMessageQueueNew (3, sizeof(SystemStatus_t), &OledStatusQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of SoilSensorTask */
  SoilSensorTaskHandle = osThreadNew(StartSoilSensorTask, NULL, &SoilSensorTask_attributes);

  /* creation of UartPrintTask */
  UartPrintTaskHandle = osThreadNew(StartUartPrintTask, NULL, &UartPrintTask_attributes);

  /* creation of PumpControlTask */
  PumpControlTaskHandle = osThreadNew(StartPumpControlTask, NULL, &PumpControlTask_attributes);

  /* creation of ButtonTask */
  ButtonTaskHandle = osThreadNew(StartButtonTask, NULL, &ButtonTask_attributes);

  /* creation of WaterTankTask */
  WaterTankTaskHandle = osThreadNew(StartWaterTankTask, NULL, &WaterTankTask_attributes);

  /* creation of SystemMonitorTa */
  SystemMonitorTaHandle = osThreadNew(StartSystemMonitorTask, NULL, &SystemMonitorTa_attributes);

  /* creation of SystemStatusTas */
  SystemStatusTasHandle = osThreadNew(StartSystemStatusTask, NULL, &SystemStatusTas_attributes);

  /* creation of OledDisplayTask */
  OledDisplayTaskHandle = osThreadNew(StartOledDisplayTask, NULL, &OledDisplayTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of SoilEvent */
  SoilEventHandle = osEventFlagsNew(&SoilEvent_attributes);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PUMP_RELAY_Pin|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PUMP_RELAY_Pin PB12 PB13 PB14 */
  GPIO_InitStruct.Pin = PUMP_RELAY_Pin|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSoilSensorTask */
/**
  * @brief  Function implementing the SoilSensorTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSoilSensorTask */
void StartSoilSensorTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	 APP_SoilSensorTask(argument);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartUartPrintTask */
/**
* @brief Function implementing the UartPrintTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartPrintTask */
void StartUartPrintTask(void *argument)
{
  /* USER CODE BEGIN StartUartPrintTask */
  /* Infinite loop */
	APP_UartPrintTask(argument);
  /* USER CODE END StartUartPrintTask */
}

/* USER CODE BEGIN Header_StartPumpControlTask */
/**
* @brief Function implementing the PumpControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPumpControlTask */
void StartPumpControlTask(void *argument)
{
  /* USER CODE BEGIN StartPumpControlTask */
  /* Infinite loop */
	APP_PumpControlTask(argument);

  /* USER CODE END StartPumpControlTask */
}

/* USER CODE BEGIN Header_StartButtonTask */
/**
* @brief Function implementing the ButtonTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartButtonTask */
void StartButtonTask(void *argument)
{
  /* USER CODE BEGIN StartButtonTask */
  /* Infinite loop */
	  APP_ButtonTask(argument);

  /* USER CODE END StartButtonTask */
}

/* USER CODE BEGIN Header_StartWaterTankTask */
/**
* @brief Function implementing the WaterTankTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWaterTankTask */
void StartWaterTankTask(void *argument)
{
  /* USER CODE BEGIN StartWaterTankTask */
  /* Infinite loop */
	APP_WaterTankTask(argument);
  /* USER CODE END StartWaterTankTask */
}

/* USER CODE BEGIN Header_StartSystemMonitorTask */
/**
* @brief Function implementing the SystemMonitorTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSystemMonitorTask */
void StartSystemMonitorTask(void *argument)
{
  /* USER CODE BEGIN StartSystemMonitorTask */
  /* Infinite loop */
	APP_SystemMonitorTask(argument);
  /* USER CODE END StartSystemMonitorTask */
}

/* USER CODE BEGIN Header_StartSystemStatusTask */
/**
* @brief Function implementing the SystemStatusTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSystemStatusTask */
void StartSystemStatusTask(void *argument)
{
  /* USER CODE BEGIN StartSystemStatusTask */
  /* Infinite loop */
	APP_SystemStatusTask(argument);
  /* USER CODE END StartSystemStatusTask */
}

/* USER CODE BEGIN Header_StartOledDisplayTask */
/**
* @brief Function implementing the OledDisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOledDisplayTask */
void StartOledDisplayTask(void *argument)
{
  /* USER CODE BEGIN StartOledDisplayTask */
  /* Infinite loop */
	APP_OledDisplayTask(argument);
  /* USER CODE END StartOledDisplayTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
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
