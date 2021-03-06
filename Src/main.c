/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stm32f1xx_hal.h"
#include <string.h>
#include "SerialTaskSend.h"
#include "SerialTaskReceive.h"
#include "getserialbuf.h"
#include "stackwatermark.h"
#include "yprintf.h"
#include "gateway_comm.h"
#include "gateway_CANtoPC.h"
#include "DTW_counter.h"
#include "yscanf.h"
#include "adctask.h"
#include "ADCTask.h"
#include "adcparams.h"
#include "adcparamsinit.h"
#include "gateway_PCtoCAN.h"
#include "morse.h"
#include "MailboxTask.h"
#include "ContactorTask.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct CAN_CTLBLOCK* pctl0;	// Pointer to CAN1 control block

uint32_t debugTX1c;
uint32_t debugTX1c_prev;

uint32_t debug03;
uint32_t debug03_prev;

extern osThreadId SerialTaskHandle;
extern osThreadId SerialTaskReceiveHandle;
extern osThreadId ADCTaskHandle;

uint16_t m_trap = 440;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void const * argument);

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
	BaseType_t ret;	   // Used for returns from function calls
	osMessageQId Qidret; // Function call return
	osThreadId Thrdret;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	DTW_counter_init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

	
  /* USER CODE END 2 */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 304);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
/* =================================================== */

	/* Create serial task (priority) */
	// Task handle "osThreadId SerialTaskHandle" is global
	Thrdret = xSerialTaskSendCreate(0);	// Create task and set Task priority
	if (Thrdret == NULL) morse_trap(17);

	/* Add bcb circular buffer to SerialTaskSend for usart1 */
	#define NUMCIRBCB1  16 // Size of circular buffer of BCB for usart6
	ret = xSerialTaskSendAdd(&huart1, NUMCIRBCB1, 1); // dma
	if (ret < 0) morse_trap(1); // Panic LED flashing

	
	/* Setup semaphore for yprint and sprintf et al. */
	yprintf_init();

	/* Create serial receiving task. */
	Thrdret = xSerialTaskReceiveCreate(1);
	if (Thrdret == NULL) morse_trap(21);

  
	/* Contactor control. */
	Thrdret = xContactorTaskCreate(1);
	if (Thrdret == NULL) morse_trap(18);

	/* ADC summing, calibration, etc. */
	Thrdret = 	xADCTaskCreate(2);
	if (Thrdret == NULL) morse_trap(20);
	
/* =================================================== */

  /* USER CODE END RTOS_THREADS */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
	m_trap = 447; // morse_trap(447);
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
	m_trap = 445; // morse_trap(445);
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 36000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */
	m_trap = 443; // morse_trap(443);
  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 14400-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
	m_trap = 442; // morse_trap(442);
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BP_green_LED_Pin|DMOC_FET_gate_driver_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(test_pin_GPIO_Port, test_pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BP_green_LED_Pin DMOC_FET_gate_driver_Pin */
  GPIO_InitStruct.Pin = BP_green_LED_Pin|DMOC_FET_gate_driver_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : test_pin_Pin */
  GPIO_InitStruct.Pin = test_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(test_pin_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	struct SERIALSENDTASKBCB* pbuf1 = getserialbuf(&huart1,96);
	if (pbuf1 == NULL) morse_trap(11);

	int ctr = 0; // Running count
	uint32_t heapsize;

// Copy of fast sum of ADC readings
int i;

// Number ADC readings per sec: 1153-1154.
extern uint32_t adcsumdb[6];// DMA sums
//extern uint32_t adcdbctr; // ADC DMA sum counter
double dt1;

extern struct CURSWFUNCTION curswfunction;
struct CURSWFUNCTION* pcf = &curswfunction;

osDelay(50); // Allow ADC task to get these initialized

// Temperature precomputed ratio check
if (pcf->padc->intern.dx25 < 0.1) morse_trap(48);

//double dxdvref = pcf->padc->intern.dvref * (1.0/4.3E-3);
if (pcf->padc->intern.dxdvref < 0.1) morse_trap(49);

// DTW time duration checks
extern uint32_t adcdbg2;

extern uint32_t dbgCE1;
uint32_t dbgCE1_prev = dbgCE1;

#define LOOPDELAY 1000

  /* Infinite loop */
  for(;;)
  {
	osDelay(LOOPDELAY);
//while( (dbggpsflag-dbggpsflag_prev) == 0);
//dbggpsflag_prev = dbggpsflag;

	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13); // LED Green

#define SHOWSTACKWATERMARK
#ifdef SHOWSTACKWATERMARK
			// Following takes 1370791 sysclock ticks 19.0 ms (includes serial port wait)
			/* Display the amount of unused stack space for tasks. */
			yprintf(&pbuf1,"\n\n\r#%4i Unused Task stack space--", ctr++);
			stackwatermark_show(defaultTaskHandle,&pbuf1,"defaultTask---");
			stackwatermark_show(SerialTaskHandle ,&pbuf1,"SerialTaskSend");
	//		stackwatermark_show(CanRxTaskHandle  ,&pbuf1,"CanRxTask-----");
			stackwatermark_show(ADCTaskHandle    ,&pbuf1,"ADCTask-------");
 		 stackwatermark_show(ContactorTaskHandle,&pbuf1,"ContactorTask-");
	stackwatermark_show(SerialTaskReceiveHandle,&pbuf1,"SerialReceiveTask");

			/* Heap usage (and test fp woking. */
  #define HEAPSIZELIST 
  #ifdef  HEAPSIZELIST
			heapsize = xPortGetFreeHeapSize();
			yprintf(&pbuf1,"\n\r#GetFreeHeapSize: total: %i free %i %3.1f%% used: %i\n\n\r",configTOTAL_HEAP_SIZE, heapsize,\
				100.0*(float)heapsize/configTOTAL_HEAP_SIZE,(configTOTAL_HEAP_SIZE-heapsize));
  #endif

#endif

#define SHOWSUMMEDADCCHANNELS
#ifdef  SHOWSUMMEDADCCHANNELS
		for (i = 0; i < 6; i++)
		{	
			yprintf(&pbuf1,"%7i ",adcsumdb[i]); // This is what routines work with
		}
		yprintf(&pbuf1, " :%7i %8.1f\n\r ", pcf->padc->intern.adcfiltemp, (double)(pcf->padc->intern.adcfilvref)/pcf->padc->intern.iiradcvref.pprm->scale);
#endif

#define SHOWEXTENDEDSUMMEDADCCHANNELS
#ifdef  SHOWEXTENDEDSUMMEDADCCHANNELS
		yprintf(&pbuf1, "\n\r     5v    cur1    cur2     12v    temp    vref\n\rA ");
		// Following loop takes about 450000 sysclock ticks 6.2 ms (includes waits for serial port)
		for (i = 0; i < 6; i++)
		{	
			yprintf(&pbuf1,"%8.1f",(double)(pcf->padc->chan[i].xsum[1])*(1.0/ADCEXTENDSUMCT));
		}
		yprintf(&pbuf1,"\n\r");
#endif

#define SHOWINTERNALTEMPERATURECALCULATIONS 
#ifdef SHOWINTERNALTEMPERATURECALCULATIONS
	/* Internal temperature computation check. */
	// The following takes 1418 sysclock ticks
	dt1 = (pcf->padc->intern.dx25 - (pcf->padc->intern.dxdvref * ((double)pcf->padc->intern.adcfiltemp / (double)pcf->padc->intern.adcfilvref )))  + pcf->padc->lc.calintern.drmtemp;

	yprintf(&pbuf1,"\n\rT degC: (doubles)%6.2f %6.2f (scaled int)%i\n\r", dt1,(double)pcf->padc->intern.itemp/(1<<ADCSCALEbits), adcdbg2,pcf->padc->intern.adccmpvref);
#endif

//      pcf->padc->chan[ADC1IDX_5VOLTSUPPLY].ival,  pcf->padc->v5.adcfil, 
//      pcf->padc->chan[ADC1IDX_12VRAWSUPPLY].ival, pcf->padc->v12.adcfil,
//#define SHOWADCDETAILS
#ifdef  SHOWADCDETAILS
	yprintf(&pbuf1,"\n\r%i %i %i %i %i %0.6f %0.4f : %0.4f\n\r",
      pcf->padc->v12.ival,
      pcf->padc->intern.adcfilvref,
      pcf->padc->intern.vref,
	   pcf->padc->v12.adcfil,
	   pcf->padc->lc.calintern.adcvdd,
      pcf->padc->v12.k,  
      (pcf->padc->v12.k * (double)pcf->padc->v12.ival * (1.0/(1<<ADCSCALEbits))), 
      pcf->padc->intern.dvref );

	yprintf(&pbuf1,"\n\rV v5: %0.3f  v12: %0.2f\n\r",
	  (pcf->padc->v5.k  * (double)pcf->padc->v5.ival  * (1.0/(1<<ADCSCALEbits))),
	  (pcf->padc->v12.k * (double)pcf->padc->v12.ival * (1.0/(1<<ADCSCALEbits))) );
#endif


#define TESTRATIOMETRICCALIBRATION
#ifdef  TESTRATIOMETRICCALIBRATION
/*
struct ADCRATIOMETRIC
{
	struct IIRFILTERL iir;    // Intermediate filter params
	double drk5ke;    // Ratio k5/ke resistor dividers ratio (~1.0)
	double drko;      // Offset ratio: double (~0.5)
	double dscale;    // Scale factor
	uint32_t adcfil;  // Filtered ADC reading
	int32_t irk5ke;   // Ratio k5/ke ratio: scale int (~32768)
	int32_t irko;     // Offset ratio: scale int (~32768)
	int32_t iI;       // integer result w offset, not final scaling
}; */
yprintf(&pbuf1,"\n\rRATIOMETRIC: struct ADCRATIOMETRIC for cur1--\n\r");
yprintf(&pbuf1,"drko   %0.5f\n\rdscale %0.6f\n\r",
	pcf->padc->cur1.drko,    /* Offset ratio: double (~0.5)               */
	pcf->padc->cur1.dscale); /* Scale factor                              */
yprintf(&pbuf1,"adcfil %i\n\rirko   %i\n\riI     %i\n\r",
	pcf->padc->cur1.adcfil,  /* Filtered ADC reading                      */
	pcf->padc->cur1.irko,    /* Offset ratio: scale int (~32768)          */
	pcf->padc->cur1.iI );    /* integer result w offset, not final scaling*/

double dI = (pcf->padc->cur1.iI * pcf->padc->cur1.dscale) / (1<<ADCSCALEbits);
yprintf(&pbuf1,"calib %0.5f\n\r",dI);

// Debug
extern uint32_t dbgadcfil;
extern uint32_t dbgadcratio;
yprintf(&pbuf1,"dadcfil %i\n\rratio  %i\n\r",
dbgadcfil,
dbgadcratio);

yprintf(&pbuf1,"icurrentdisconnect %i iI %d\n\r",pcf->icurrentdisconnect,pcf->padc->cur1.iI);

#endif

#define TESTABSOLUTECALIBRATION
#ifdef  TESTABSOLUTECALIBRATION

double dt1 = ((double)pcf->padc->v5.ival * (1.0/(1<<ADCSCALEbits)) * pcf->padc->v5.dscale );
yprintf(&pbuf1,"v5--\n\radc v5  %i\n\rival    %i\n\rk       %8.6f\n\rdref   %9.6f\n\r",
pcf->padc->v5.adcfil,
pcf->padc->v5.ival,
pcf->padc->v5.k,
pcf->padc->intern.dvref);
yprintf(&pbuf1,"cmpvref %i\n\radcvref %i\n\rdscale  %8.6f\n\r5V %13.3f\n\r",
pcf->padc->intern.adccmpvref,
pcf->padc->intern.adcfilvref,
pcf->padc->v5.dscale,
dt1);

dt1 = ((double)pcf->padc->v12.ival * (1.0/(1<<ADCSCALEbits)) * pcf->padc->v12.dscale );
yprintf(&pbuf1,"v12--\n\radc v12 %i\n\rival    %i\n\rk       %8.6f\n\rdref   %9.6f\n\r",
pcf->padc->v12.adcfil,
pcf->padc->v12.ival,
pcf->padc->v12.k,
pcf->padc->intern.dvref);
yprintf(&pbuf1,"cmpvref %i\n\radcvref %i\n\rdscale  %8.6f\n\r12V %12.3f\n\r",
pcf->padc->intern.adccmpvref,
pcf->padc->intern.adcfilvref,
pcf->padc->v12.dscale,
dt1);

#endif

  } // END OF FOR LOOP

  /* USER CODE END 5 */
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
  if (htim->Instance == TIM2) {
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
	morse_trap(m_trap); // Trap any HAL Init

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
morse_trap(71);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
