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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// WS2812B (LED) Expects GRB
typedef union{
	struct{
		uint8_t b;
		uint8_t r;
		uint8_t g;
	}color;
	uint32_t data; // BRG and "data" share the same memory space. This allows us to access the pixel data with ease
}PixelRGB_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Note that the layout is different between the left and the right tail lights
#define LEFT_NUMPIXEL 107 // Number of Pixel for the in Total for Left Tail Light
#define LEFT_CUTOFF 76    // Number of Pixel for the Brake/Default Signal (0-76 Red)
// The Rest are Used for Turn/Hazard (Yelllow) 76-107 (exclude 107)
#define LEFT_DMABUF_LEN (LEFT_NUMPIXEL*24) + 100


#define RIGHT_NUMPIXEL 120 // Number of Pixel for the Upper Portion for the Right Tail Light (Turn/Hazard Signal)
#define RIGHT_CUTOFF 80 // Number of Pixel for the Brake/Default Signal (40-120, 120 exclusvie RED)
// 0-40, 40 exclusive are the pixels for the Turn/Hazard (Yellow) Signals
#define RIGHT_DMABUF_LEN (RIGHT_NUMPIXEL * 24) + 100 // 100 Bits Corresponds to 100*0.96us of Lows


#define DASHLIGHT_ID 5
#define BRAKEBOARD_ID 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch1_trig;
DMA_HandleTypeDef hdma_tim3_ch4_up;

/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int left_updateFlag;
int right_updateFlag;
PixelRGB_t Left_PixelData[LEFT_NUMPIXEL];
PixelRGB_t Right_PixelData[RIGHT_NUMPIXEL];
// DMA Transfers Requires Pointer. We will create that locally later
uint8_t left_dma_Buffer[LEFT_DMABUF_LEN];
uint8_t right_dma_Buffer[RIGHT_DMABUF_LEN];

uint8_t blinkData;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	// Extract the LED status update bits
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

	if(RxHeader.StdId == DASHLIGHT_ID){
		// Dashboard only controls the blinking of the lights
		// Hazard or Left or Right
		blinkData = RxData[0];
	}
	else if(RxHeader.StdId = BRAKEBOARD_ID){
		// Brakeboard only controls the Red portion of the lights
		// Bright Red / Dim Red
		brakeData = RxData[0];
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){

// PICK ONE: Found out which one it is (most likely DMA one)
  // Maybe check HAL_TIM_ChannelStateTypeDef to see which one finished?
  if(htim->ChannelState[0] == HAL_TIM_CHANNEL_STATE_READY){
	  HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
  }
  if(htim->ChannelState[3] == HAL_TIM_CHANNEL_STATE_READY){
 	  HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_4);
   }

  // Either the above or check DMA1 channel 6 -> TIM channel 1
  // DMA1 channel 3 -> TIM channel 4
  if(htim->hdma[5]->State ==  HAL_DMA_STATE_READY){
	  HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
  }
  if(htim->hdma[2]->State == HAL_DMA_STATE_READY){
	  HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_4);

  }
  datasentflag = 1;


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
  MX_CAN_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // Prepares the CAN hardware with the existing configuration and start allowing
  // CAN message Transmitting and Receiving
  HAL_CAN_Start(&hcan);

  // Triggers Interrupt whenever FIFO0 receive a new message
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);


  left_updateFlag = 0;
  right_updateFlag = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(left_updateFlag == 1){

	  }
	  if(right_updateFlag == 1){

	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_FilterTypeDef canfilterconfig;
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0;
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = DASHLIGHT_ID << 5; // CAN frame ID is 11 bit, but it's fetched as a 16 bit
  	  	  	  	  	  	  	  	  	  	  	  	  	// Actual CAN id is the fetched one right shifted by 5
  	  	  	  	  	  	  	  	  	  	  	  	  	// However, to accept it, we might create a filter that
  	  	  	  	  	  	  	  	  	  	  	  	  	// Matches the ID and shifts it left by 5 bits
  canfilterconfig.FilterIdLow = BRAKEBOARD_ID << 5;
  canfilterconfig.FilterMaskIdHigh = 0xFFFF;
  canfilterconfig.FilterMaskIdLow = 0xFFFF;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDLIST;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_16BIT;
  canfilterconfig.SlaveStartFilterBank = 0;
  // Sets up the Filter Hardware
  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 57;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  HAL_GPIO_WritePin(LED_CAN_GPIO_Port, LED_CAN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_CAN_Pin */
  GPIO_InitStruct.Pin = LED_CAN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_CAN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
