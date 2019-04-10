/* USER CODE BEGIN Header */

/**
 * mxusartusbcan.c
 * 02/04/2019
 * Debug & test routines in ../Ourwares directory
*/

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "SerialTaskSend.h"
#include "stm32f4xx_hal_pcd.h"
#include "usbd_cdc_if.h"
#include "cdc_txbuff.h"
#include "CanTask.h"
#include "can_iface.h"
#include "canfilter_setup.h"
#include "stm32f4xx_hal_can.h"
#include "getserialbuf.h"
#include "SerialTaskSend.h"
#include "stackwatermark.h"
#include "yprintf.h"
#include "gateway_comm.h"
#include "gateway_CANtoPC.h"
#include "DTW_counter.h"
#include "SerialTaskReceive.h"
#include "yscanf.h"
#include "adctask.h"
#include "gateway_PCtoCAN.h"
#include "morse.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void* verr[8];
uint32_t verrx = 0;
__attribute__( ( always_inline ) ) __STATIC_INLINE uint32_t __get_SP(void) 
{ 
  register uint32_t result; 

  __ASM volatile ("MOV %0, SP\n" : "=r" (result) ); 
  return(result); 
} 

uint32_t timectr = 0;
struct CAN_CTLBLOCK* pctl1;	// Pointer to CAN1 control block
struct CAN_CTLBLOCK* pctl2;	// Pointer to CAN2 control block

uint32_t debugTX1b;
uint32_t debugTX1b_prev;

uint32_t debugTX1c;
uint32_t debugTX1c_prev;

uint32_t debug03;
uint32_t debug03_prev;

extern osThreadId SerialTaskHandle;
extern osThreadId CanTxTaskHandle;
extern osThreadId CanRxTaskHandle;
extern osThreadId SerialTaskReceiveHandle;

uint8_t canflag;
uint8_t canflag1;
uint8_t canflag2;

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

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osTimerId myTimer01Handle;
osMutexId myMutex01Handle;
osSemaphoreId myBinarySem01Handle;
osSemaphoreId myCountingSem01Handle;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
osThreadId CanTask01Handle;
//osSemaphoreId vsnprintfSemaphoreHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void Callback01(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void StartCanTask01(void const * argument);
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
	BaseType_t ret;
//	struct CAN_CTLBLOCK* pcbret;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	DTW_counter_init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

/*
DiscoveryF4 LEDs --
 GPIOD, GPIO_PIN_12 GREEN
 GPIOD, GPIO_PIN_13 ORANGE
 GPIOD, GPIO_PIN_14 RED
 GPIOD, GPIO_PIN_15 BLUE
*/
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of myMutex01 */
  osMutexDef(myMutex01);
  myMutex01Handle = osMutexCreate(osMutex(myMutex01));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of myBinarySem01 */
  osSemaphoreDef(myBinarySem01);
  myBinarySem01Handle = osSemaphoreCreate(osSemaphore(myBinarySem01), 1);

  /* definition and creation of myCountingSem01 */
  osSemaphoreDef(myCountingSem01);
  myCountingSem01Handle = osSemaphoreCreate(osSemaphore(myCountingSem01), 2);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */

//	osSemaphoreDef(sprintfSemaphore);
//   vsnprintfSemaphoreHandle = osSemaphoreCreate(osSemaphore(sprintfSemaphore), 1);

  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of myTimer01 */
  osTimerDef(myTimer01, Callback01);
  myTimer01Handle = osTimerCreate(osTimer(myTimer01), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityIdle, 0, 384);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, StartTask03, osPriorityIdle, 0, 300);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
/* =================================================== */
	/* Create serial task (priority) */
	// Task handle "osThreadId SerialTaskHandle" is global
	xSerialTaskSendCreate(0);	// Create task and set Task priority

	/* Add bcb circular buffer to SerialTaskSend for usart6 */
	#define NUMCIRBCB6  16 // Size of circular buffer of BCB for usart6
	ret = xSerialTaskSendAdd(&huart6, NUMCIRBCB6, 0); // char-by-char
	if (ret < 0) morse_trap(1); // Maybe add panic led flashing here?

	/* Add bcb circular buffer to SerialTaskSend for usart2 */
	#define NUMCIRBCB2  12 // Size of circular buffer of BCB for usart2
	ret = xSerialTaskSendAdd(&huart2, NUMCIRBCB2, 1); // dma
	if (ret < 0) morse_trap(2); // Maybe add panic led flashing here?

	/* Setup semaphore for yprint and sprintf et al. */
	yprintf_init();

	/* Create serial receiving task of uart6 (char-by-char) */
	xSerialTaskReceiveCreate(0);

	/* USB-CDC buffering */
	#define NUMCDCBUFF 3	// Number of CDC task local buffers
	#define CDCBUFFSIZE 64*16	// Best buff size is multiples of usb packet size
	struct CDCBUFFPTR* pret;
	pret = cdc_txbuff_init(NUMCDCBUFF, CDCBUFFSIZE); // Setup local buffers
	if (pret == NULL) morse_trap(3);
	
	/* USB-CDC queue and task creation */
	osMessageQId Qidret = xCdcTxTaskSendCreate(3);
	if (Qidret < 0) morse_trap(4); // Maybe add panic led flashing here
	
	/* Start software timer used for usb-cdc initial testing, and Callback01 */
	osTimerStart (myTimer01Handle, 500);

  /* definition and creation of CanTask01 */
  osThreadDef(CanTask01, StartCanTask01, osPriorityNormal, 0, 256);
  CanTask01Handle = osThreadCreate(osThread(CanTask01), NULL);
//	vTaskPrioritySet( CanTask01Handle, 3);

  /* definition and creation of CanTxTask - CAN driver TX interface. */
  Qidret = xCanTxTaskCreate(0, 32); // CanTask priority, Number of msgs in queue
	if (Qidret < 0) morse_trap(5); // Maybe add panic led flashing here?

  /* definition and creation of CanRxTask - CAN driver RX interface. */
  Qidret = xCanRxTaskCreate(1, 32); // CanTask priority, Number of msgs in queue
	if (Qidret < 0) morse_trap(6); // Maybe add panic led flashing here?

	/* Setup TX linked list for CAN1  */
	pctl1 = can_iface_init(&hcan1, 64);
	if (pctl1 == NULL) morse_trap(7); // Maybe add panic led flashing here?

	/* Setup TX linked list for CAN2  */
	pctl2 = can_iface_init(&hcan2, 8);
	if (pctl2 == NULL) morse_trap(8); // Maybe add panic led flashing here?

	/* Setup CAN hardware filters to default to accept all ids. */
	HAL_StatusTypeDef Cret;
	Cret = canfilter_setup_first(1, &hcan1, 15);
	if (Cret == HAL_ERROR) morse_trap(9);

//	Cret = canfilter_setup_first(2, &hcan2, 15);
//	if (Cret == HAL_ERROR) morse_trap(10);

	/* Remove "accept all" and add specific id & mask, or id here. */
	// See canfilter_setup.h


	/* Select interrupts for CAN1 */
	HAL_CAN_ActivateNotification(&hcan1, \
		CAN_IT_TX_MAILBOX_EMPTY     |  \
		CAN_IT_RX_FIFO0_MSG_PENDING |  \
		CAN_IT_RX_FIFO1_MSG_PENDING    );

	/* Start CAN1 */
	HAL_CAN_Start(&hcan1);
/* =================================================== */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 4;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
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
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 12;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 2000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*********************** CanTask01 ************************************************
** Set to Idle (-3) FreeRTOS priority
** List stack usage of tasks
** Test yprintf and sending with multiple serial buffers to one uart
** Test floating point linking and working
***********************************************************************************/
void StartCanTask01(void const * argument)
{
	struct SERIALSENDTASKBCB* pbuf1 = getserialbuf(&huart6,96);
	if (pbuf1 == NULL) morse_trap(11);

	struct SERIALSENDTASKBCB* pbuf2 = getserialbuf(&huart6,96);
	if (pbuf1 == NULL) morse_trap(12);

	int ctr = 0; // Running count

	double pi = 3.1415926535897932; // Test that floating pt is working

	uint32_t heapsize;

#define LOOPDELAYTICKS (64*8*5)	// 5 sec Loop delay (512 Hz tick rate)
	for ( ;; )
	{
		osDelay(LOOPDELAYTICKS);
		ctr += 1;

//		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15); // BLUE LED

		/* Old way of outputting a line. */
#ifdef OLDWAYPRINTF
		vSerialTaskSendWait(pbuf1, &noteval); // Wait for buffer to be released
		xSemaphoreTake( vsnprintfSemaphoreHandle, portMAX_DELAY ); // Protect snprintf
		pbuf1->size = snprintf((char*)pbuf1->pbuf,BUF1Z,"\n\rCT %3d ------- %f",ctr, pi*ctr);
		if (pbuf1->size > BUF1Z) pbuf1->size = BUF1Z; // snprintf buffer max limitation
		xSemaphoreGive( vsnprintfSemaphoreHandle );	// Release snprintf
		vSerialTaskSendQueueBuf(&pbuf1, &noteval); // Place control block on queue for sending
#endif

#ifdef NEWWAYPRINTF
		/* New way of setting up lines using 'yprintf'.  NOTE different struct used for buffer. */
		yprintf(&pbuf2,"\n\rctr %3d  pi/ctr: %f",ctr, pi*ctr); // Does everything.

		/* CAN msgs are counted in another task. */
		yprintf(&pbuf2,"\n\r\t\tCan Message Ct %i %f per sec",\
		(unsigned int)(debug03-debug03_prev),\
		(double)(debug03-debug03_prev)*1000.0/LOOPDELAYTICKS);
		debug03_prev = debug03;
#endif


		/* Display the amount of unused stack space for tasks. */
#define USESTACKMARK
#ifdef USESTACKMARK
		stackwatermark_show(defaultTaskHandle,&pbuf2,"defaultTask--");
		stackwatermark_show(CanTask01Handle  ,&pbuf2,"Task01-------");
		stackwatermark_show( myTask02Handle  ,&pbuf2,"Task02-------");
		stackwatermark_show( myTask03Handle  ,&pbuf2,"Task03-------");
		stackwatermark_show(SerialTaskHandle ,&pbuf2,"SerialTask---");
		stackwatermark_show(CanTxTaskHandle  ,&pbuf2,"CanTxTask----");
		stackwatermark_show(CanRxTaskHandle  ,&pbuf2,"CanRxTask----");
		stackwatermark_show(SerialTaskReceiveHandle  ,&pbuf2,"SerialRcvTask");
#endif

	/* Heap usage */
	heapsize = xPortGetFreeHeapSize();
	yprintf(&pbuf2,"\n\rGetFreeHeapSize: %i used: %i",heapsize,(configTOTAL_HEAP_SIZE-heapsize));

	/* Testing fp working */
	yprintf (&pbuf2,"\n\r\t\t\t\t\typrintf %f test", pi/ctr); // Just a test of uart2

	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
*********************** DefaultTask *******************************************
** Test usb-cdc (ttyACM0) sending
** Test sending fixed CAN msg
*******************************************************************************
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 5 */
	
	/* Wait for PC to recognize ACM0 (USB-CDC) */
	osDelay(1000);

#ifdef USEACMOSERIALSENDING
/* Testing usb-cdc w minicom ttyACM0 */
char c[64];
struct CDCTXTASKBCB cdc1;
cdc1.pbuf = (uint8_t*)&c[0];

xSemaphoreTake( vsnprintfSemaphoreHandle, portMAX_DELAY );
	cdc1.size = snprintf(c,64,"\n\n\rmxusartusbcan: initial line with usb-cdc ACM0 %i\n\n\r",(unsigned int)timectr++);
	if (cdc1.size > 64) cdc1.size = 64;
xSemaphoreGive( vsnprintfSemaphoreHandle ); 

xQueueSendToBack(CdcTxTaskSendQHandle,&cdc1,5000);
#endif


	/* Assign notification bits for this task */
	#define TSKDEFBUFBIT02 0x4	// Assign notification bit position to ADC ???

	/* FreeRTOS notifications to this task sets bits in this word. */
//26	uint32_t noteval = 0; 

	/* Malloc a serial buffer control block and buffer. Initial noteval. */
//2	struct SERIALSENDTASKBCB* pbuf1 = getserialbuf(&huart2,64);
//6	struct SERIALSENDTASKBCB* pbuf2 = getserialbuf(&huart6,32);

	int ctr = 0;

	/* Test CAN msg */
	struct CANTXQMSG testtx;
	testtx.pctl = pctl1;
	testtx.can.id = 0x12200000;
	testtx.can.dlc = 8;
	testtx.can.cd.uc[0] = 0x01;
	int i;
	for (i = 1; i< 8; i++)
	{
		testtx.can.cd.uc[i] = testtx.can.cd.uc[i-1] + 0x22;
	}

	testtx.maxretryct = 8;
	testtx.bits = 0;
	
  /* Infinite loop */
  for(;;)
  {
    osDelay(8*4);	// Four loops per second

HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); // GREEN

	/* USB-CDC */
#ifdef USEACMOSERIALSENDING
xSemaphoreTake( vsnprintfSemaphoreHandle, portMAX_DELAY );
	cdc1.size = sprintf(c,"DefaultTask: USB-CDC output on /dev/ttyACM0. %i\n\r",ctr);
xSemaphoreGive( vsnprintfSemaphoreHandle );
	xQueueSendToBack(CdcTxTaskSendQHandle,&cdc1,5000);
	mCdcTxQueueBuf(&cdc1);
	mCdcTxQueueBuf(&cdc1);
	mCdcTxQueueBuf(&cdc1);
	mCdcTxQueueBuf(&cdc1);
	mCdcTxQueueBuf(&cdc1);
#endif

	/* Some usart testing with usart2. */
//2	yprintf(&pbuf1,"\n\r\t\tDefaultTask: %3i",ctr);

	/* Some usart testing with usart6. */
//6	yprintf(&pbuf2,"\n\r\t\tDefaultTask: %3i",ctr);

	ctr += 1;

/* ==== CAN MSG sending test ===== */
	/* Place test CAN msg to send on queue in a burst. */
	/* Note: an odd makes the LED flash since it toggles on each msg. */
	for (i = 0; i < 7; i++)
		xQueueSendToBack(CanTxQHandle,&testtx,portMAX_DELAY);
  }

  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*********************** Task02 ************************************************
** Test sscanf: incoming ascii on usart6
 * Test ADC: three inputs plus internal temperature and voltage reference
 * Multiple bits on TastNotifyWait plus yprintf
*******************************************************************************
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
	char* pline;	// Pointer to line buffer

	#define TSK02BIT00	(1 << 0)  // Task notification bit for serial input (SerialTaskReceive.c)
	#define TSK02BIT02	(1 << 2)  // Task notification bit for ADC dma 1st 1/2 (adctask.c)
	#define TSK02BIT03	(1 << 3)  // Task notification bit for ADC dma end (adctask.c)
	#define TSK02BIT04	(1 << 4)  // Task notification bit for huart2 incoming ascii CAN

/* Notefication on multiple bits-- 

The xTaskNotifyWait resets the bits in internal notification word for this task 
   when the xTaskNotifyWait is entered.  

When there is a notification, the process calling the NotifyGive specifies the 
   bit that is ORed into the notification word, and the notification word internal 
   to the FreeRTOS is copied into the 'noteval'.

xTaskNotifyWait exits.  'noteused' is set to zero and the subsequent processing 
   checks the bits in 'noteval' as they are handled.  

Bits in 'noteused' are fed back into xTaskNotifyWait, which resets the bits that 
   were handled.  If other bits in 'noteval' were turned on by a notification during 
   the processing of a notification, 'noteused' will not turn them off, and the 
   'Wait will return immediately.
*/

	/* A notification copies the internal notification word to this. */
	uint32_t noteval = 0;    // Receives notification word upon an API notify

	/* notification bits processed after a 'Wait. */
	uint32_t noteused = 0;

	/* Setup serial receive for uarts */
	struct SERIALRCVBCB* pbcb;		// usart6 (PC minicom->yscanf test)
	struct SERIALRCVBCB* prbcb2;	// usart2 (PC->CAN msg test)

	/* Setup serial input buffering and line-ready notification */
   //   (ptr uart handle, dma flag, notiification bit, 
   //   ptr notification word, number line buffers, size of lines, 
   //   dma buffer size);

	/* PC-to-CAN ascii/hex incoming "lines" directly converts to CAN msgs. */
	prbcb2 = xSerialTaskRxAdduart(&huart2,1,TSK02BIT04,\
		&noteval,12,32,128,1); // buff 12 CAN, of 24 bytes, 192 total dma, /CAN mode
	if (prbcb2 == NULL) morse_trap(13);

	/* Incoming ascii lines. */
	pbcb  = xSerialTaskRxAdduart(&huart6,1,TSK02BIT00,\
		&noteval,3,96,48,0);	// 3 line buffers of 96 chars, 48 total dma, line mode
	if (pbcb == NULL) morse_trap(14);

	struct CANRCVBUFPLUS* pcanp;  // Basic CAN msg Plus error and seq number

	/* Setup serial output for uart. */
	struct SERIALSENDTASKBCB* pbuf21 = getserialbuf(&huart6,96);
	struct SERIALSENDTASKBCB* pbuf24 = getserialbuf(&huart6,24);
	struct SERIALSENDTASKBCB* pbuf25 = getserialbuf(&huart6,64);
	struct SERIALSENDTASKBCB* pbuf26 = getserialbuf(&huart6,64);
	struct SERIALSENDTASKBCB* pbuf27 = getserialbuf(&huart6,64);
	struct SERIALSENDTASKBCB* pbuf28 = getserialbuf(&huart6,128);

osDelay(10); // Some debugging & testing

/* sscanf testing */
double d1 = 0;
double d2 = 0;
int64_t nn = 0;

yscanf_init(); // Get semaphore for sscanf

// 3.1415926535897932 3.1415926535897932 10 

/*
uint16_t* adctask_init(ADC_HandleTypeDef* phadc,\
	uint32_t  notebit1,\
	uint32_t  notebit2,\
	uint32_t* pnoteval,\
	uint16_t  dmact)
*/
	/* Get buffers, "our" control block, and start ADC/DMA running. */
	struct ADCDMATSKBLK* pblk = adctask_init(&hadc1,TSK02BIT02,TSK02BIT03,&noteval,16);
	if (pblk == NULL) {HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15,GPIO_PIN_SET); morse_trap(15);}

/* Counts of ADC/DMA interrupts */
//extern uint32_t Ddma1;
extern uint32_t Ddma2;
//uint32_t Ddma1_prev = 0;
uint32_t Ddma2_prev = 0;

uint32_t canflag1_prev = 0;
uint32_t canflag2_prev = 0;

uint32_t dbgT0;
uint32_t dbgT1;

int i;
int n = pblk->phadc->Init.NbrOfConversion;
uint64_t adc[5] = {0};
float fadc[5];
uint64_t* psum;
uint16_t sumct = 0;
#define AD 13	// Scale: 16 in seq, plus sumct

	/* PC-to-CAN msg */
	struct CANTXQMSG testtx;
	testtx.pctl = pctl1;

	for ( ;; )
	{
		/* Wait for line completion */
		xTaskNotifyWait(noteused, 0, &noteval, portMAX_DELAY);
		noteused = 0;	// Accumulate bits in 'noteval' processed.

		psum = adctask_sum(pblk);	// Sum 1/2 dma buffer 
		if (psum != NULL)
		{
			/* We handled one, or both, noteval bits */
			noteused |= (pblk->notebit1 | pblk->notebit2);

			/* Sum so more to reduce output rate, and further filter. */
				for (i = 0; i < n; i++)
				{
					adc[i] += *(psum+i);
					*(psum+i) = 0;
				}
				sumct += 1;
				if (sumct >= (1<<9))
				{
					for (i = 0; i < n; i++)
					{
						fadc[i] = adc[i];
						fadc[i] /= (1<<AD);
					}
					yprintf(&pbuf26,"\n\rS: %6.1f %6.1f %6.1f %6.1f %6.1f",fadc[0],fadc[1],fadc[2],fadc[3],fadc[4]);
//					yprintf(&pbuf26,"\n\rS: %4lli %4lli %4lli %4lli %4lli",adc[0]>>AD,adc[1]>>AD,adc[2]>>AD,adc[3]>>AD,adc[4]>>AD);
					adc[0] = 0;adc[1] = 0; adc[2] = 0; adc[3] = 0; adc[4] = 0;  
					sumct = 0;
				}
		}

		/* Display number of 1st 1/2 DMA interrupts per second. */
		if ((noteval & TSK02BIT02) != 0)
		{ // Here, 1st half of DMA ready

			if (canflag1 != canflag1_prev)
			{ // DMA interrupts during one second
				canflag1_prev = canflag1;
				yprintf(&pbuf25,"\n\rD1 %6i %6i %6i %6i %6i",*(pblk->pdma1+0),*(pblk->pdma1+1),*(pblk->pdma1+2),*(pblk->pdma1+3),*(pblk->pdma1+4) );
//				Ddma1_prev = Ddma1;
			}
		}

		/* Display number of 2nd 1/2 DMA interrupts per second. */
		if ((noteval & TSK02BIT03) != 0)
		{ // Here, 2nd half of DMA ready
			if (canflag2 != canflag2_prev)
			{ // DMA interrupts during one second
				canflag2_prev = canflag2;
				yprintf(&pbuf24,"\n\rDMA2 %d",Ddma2-Ddma2_prev);
				Ddma2_prev = Ddma2;
			}
		}

		/* Handle an incoming uart line */
		if ((noteval & TSK02BIT00) != 0)
		{ // Here, one or more input lines ready
			noteused |= TSK02BIT00; // We handled the bit
			do
			{
				/* Get pointer of next completed line. */
				pline = xSerialTaskReceiveGetline(pbcb);
				if (pline != NULL)
				{ // Here, a line is ready.
dbgT0=DTWTIME;
					/* 4505 cycles: 26.8 us */
				  yprintf(&pbuf21, "\n\rHELLO from task02: 0X%08X :%d |%s",pline,strlen(pline), pline);
dbgT1=DTWTIME - dbgT0;

/* NOTE: It isnecessary to remove '-specs=nano.specs' from linker flags in 'Makefile'
   to load library functions for scanf. */

				/* Approx 4200 to 14000 cycles: 25 -  83 us */
				yscanf(pline,"%lf %lf %lli", &d1,&d2,&nn);

				/* Behold what has been wrought!  1101 cycles: 6.6 us */
				yprintf(&pbuf27,"\n\rhell yes: %21.16lf %lf %21.16lf %lli %i",d1,d2,d1*d2*nn,nn,(unsigned int)(dbgT1));

				}
			} while (pline != NULL);
		}

		/* Handle incoming usart2 carrying ascii/hex CAN msgs */
		if ((noteval & TSK02BIT04) != 0)
		{ // Here, one or more CAN msgs have been received
			noteused |= TSK02BIT04; // We handled the bit

			/* Get incoming CAN msgs from PC and queue for output to CAN bus. */
			do
			{
				pcanp = gateway_PCtoCAN_getCAN(prbcb2);
				if (pcanp != NULL)
				{
					/* Check for errors */
					if (pcanp->error == 0)
					{
						/* Place CAN msg on queue for sending to CAN bus */
						testtx.can = pcanp->can;
						xQueueSendToBack(CanTxQHandle,&testtx,portMAX_DELAY);
					}
					else
					{ // Here, one or more errors. List for the hapless Op to ponder
						yprintf(&pbuf28,"\n\r@@@@@ PC CAN ERROR: %i 0X%04X, 0X%08X 0X%02X 0X%08X %i 0X%02X 0X%02X %s",pcanp->seq, pcanp->error,\
							pcanp->can.id,pcanp->can.dlc,pcanp->can.cd.ui[0]);

						/* For test purposes: Place CAN msg on queue for sending to CAN bus */
						testtx.can = pcanp->can;
						xQueueSendToBack(CanTxQHandle,&testtx,portMAX_DELAY);
					}
				}
			} while ( pcanp != NULL);
		}
	}
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
********************* Task03 **************************************************
** Receive CAN msgs placed on queue by CAN callback in 'iface.c'
*******************************************************************************
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */

	/* Get buffer with buffer control block and semaphore for use with 'yprintf' */
	struct SERIALSENDTASKBCB* pbuf1 = getserialbuf(&huart6,128);
	struct SERIALSENDTASKBCB* pbuf3 = getserialbuf(&huart2,128);

	/* Testing CAN rcv */
   BaseType_t Qret;	// queue receive return
	struct CANRCVBUFN ncan;

	int rctr = 0;	

// Debuggiing
uint32_t debugctr = 0;
uint32_t debugctr_prev = 0;
extern uint32_t debug1;
uint32_t debug1prev=debug1;

//osDelay(6000); // Debug delay

// DTW Cycles max for--
// 9498508 1328 gateway_comm_CANtoPC
//    2108  461 vSerialTaskSendWait (buffer wait not needed)
//     904  288 gateway_CANtoPC 
//    2016  961 vSerialTaskSendQueueBuf
// 7986270 1016 gateway_CANtoPC + Wait + Queue

  /* Infinite loop */
  for(;;)
  {
		Qret = xQueueReceive(CanRxQHandle,&ncan,portMAX_DELAY);
		if (Qret == pdPASS)
		{
			rctr += 1;
debug03 += 1;	// CAN msg count for Task 01 output

		/* Convert binary CAN to ASCII/HEX format for PC */
			/* Version using PC_gateway_comm and USB_PC_gateway */

		/* Original version w mods for loading queue */
//			xSemaphoreTake(pbuf3->semaphore, 0);
//			gateway_comm_CANtoPC(&pbuf3, &ncan.can);

		/* Newer version with limited options but better efficiency. */
	xSemaphoreTake(pbuf3->semaphore, 5000);
		gateway_CANtoPC(&pbuf3, &ncan.can);
	vSerialTaskSendQueueBuf(&pbuf3); // Place on queue

			if ((ncan.can.id == 0x00400000) && (ncan.can.cd.uc[0] == 0))
			{
				/* One second GPS CAN msg flags for other tasks. */
				canflag = 1;
				canflag1 += 1;
				canflag2 += 1;

#define UART2INTERFERENCE  // Test that usart6 not giving usart2 problems
#ifdef UART2INTERFERENCE
			/* Upon GPS CAN msg, 1/64 tick == 0, output received CAN msg count. */
				yprintf(&pbuf1,"\n\r\t\t=====> R %i %i %i %i %i", rctr,\
					(unsigned int)(debug1-debug1prev),\
					(unsigned int)(debugctr-debugctr_prev),\
					(unsigned int)(debugTX1b - debugTX1b_prev),\
					(unsigned int)(debugTX1c - debugTX1c_prev));

				/* Reset count differences. */
				rctr = 0;
				debug1prev     = debug1;
				debugctr_prev  = debugctr;
				debugTX1b_prev = debugTX1b;
				debugTX1c_prev = debugTX1c;

				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); // RED LED
#endif
			}
		}
		else
		{ // Here (Qret != pdPASS)
			debugctr += 1;
		}
	}

  /* USER CODE END StartTask03 */
}

/* Callback01 function */
void Callback01(void const * argument)
{
  /* USER CODE BEGIN Callback01 */

  /* USER CODE END Callback01 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
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
	verr[verrx++] = (void*)__get_SP();
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
		morse_trap(16);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
