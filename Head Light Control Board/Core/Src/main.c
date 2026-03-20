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
#include <limits.h>
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
//#define TESTING

#define NUM_PIXEL 76
#define CUT_OFF 44

#define DMABUF_LEN (NUM_PIXEL * 24) + 100
#define DASHLIGHT_ID 5

#define NEOPIXEL_ZERO 19
#define NEOPIXEL_ONE 38

const uint8_t OFF_COLOR[] = {0,0,0};
const uint8_t HEADLIGHT_COLOR[] = {255,255,255};
const uint8_t TURNSIG_COLOR[] = {255,20,0};



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch1_trig;
DMA_HandleTypeDef hdma_tim3_ch3;

/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef RxHeader; // Struct Used to Store CAN Message Received
uint8_t RxData[8];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void updateLight();
void updateDash(uint8_t blinkData_local); // Ensures when we return to function, we finish updating using the old value
void SetPixelColor(PixelRGB_t* p,const uint8_t color[]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t datasentFlag; // datasentFlag = 2 means both DMA are available, 0 means none are available
uint8_t updateDashFlag;
PixelRGB_t Left_PixelData[NUM_PIXEL] = {0};
uint32_t left_dma_Buffer[DMABUF_LEN] = {0};

PixelRGB_t Right_PixelData[NUM_PIXEL] = {0};
uint32_t right_dma_Buffer[DMABUF_LEN] = {0};

uint8_t blinkData;
uint8_t prev_blinkData;
uint32_t* pBuff_Left; // Used to point to the dma_buffer to ease increment in size of words
uint32_t* pBuff_Right;
uint8_t LEFT_BLINK;
uint8_t LEFT_BLINK_FLAG;
uint8_t RIGHT_BLINK;
uint8_t RIGHT_BLINK_FLAG;

int counter;

void SetPixelColor(PixelRGB_t* p, const uint8_t color[]){
	(*p).color.r = color [0];
	(*p).color.g = color [1];
	(*p).color.b = color [2];
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	// Extract the LED status update bits
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	if(datasentFlag & 0b00000011){
		if(RxHeader.StdId == DASHLIGHT_ID){
			// Dashboard Controls the HeadLights, and Blinking for the Headlights
			blinkData = RxData[0] & 0b00001011; // Mask out Hazard
			updateDashFlag = 1;
		}
	}
}
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){

  if( htim == &htim3){ // Check if address pointed to is the same as the address of htim3
	  // Note that TIM_Channel 3 corresponds to hdma[3], it's not zero-indexed (DMA)
	  // TIM channel are zero-indexed as stated in TIM_DMADelayPulseCplt 	   (TIM Status)
	  if(htim->hdma[1]->State ==  HAL_DMA_STATE_READY || htim->ChannelState[0] == HAL_TIM_CHANNEL_STATE_READY){
		  HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
		  datasentFlag |= 0b00000001;
	  }
	  if(htim->hdma[3]->State == HAL_DMA_STATE_READY || htim->ChannelState[2] == HAL_TIM_CHANNEL_STATE_READY){

		  HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_3);
		  datasentFlag |= 0b00000010;

	  }
#ifdef TESTING
			uint32_t left_remaining;
			uint32_t right_remaining;
		  left_remaining = htim->hdma[1]->Instance->CNDTR;
		  right_remaining = htim->hdma[3]->Instance->CNDTR;
#endif

  }
}
void LightsInit(){ // Default LED Configuration
	datasentFlag = 0;
	for(int k = 0; k < NUM_PIXEL; k++){
		SetPixelColor(&Left_PixelData[k], OFF_COLOR);
		SetPixelColor(&Right_PixelData[k], OFF_COLOR);
	}
	updateLight();
}

void updateLight(){
	datasentFlag = 0;
	pBuff_Left = left_dma_Buffer;
	pBuff_Right = right_dma_Buffer;
	for(int i = 0; i< NUM_PIXEL; i++){
		for(int j = 23; j >= 0; j--){
			if((Left_PixelData[i].data>>j) & 0x01){
				*pBuff_Left = NEOPIXEL_ONE;
			}
			else{
				*pBuff_Left = NEOPIXEL_ZERO;
			}
			pBuff_Left++;

			if((Right_PixelData[i].data>>j) & 0x01){
				*pBuff_Right = NEOPIXEL_ONE;
			}
			else{
				*pBuff_Right = NEOPIXEL_ZERO;
			}
			pBuff_Right++;
		}

	}
	for(int z = 1; z <= 100; z++){
		left_dma_Buffer[DMABUF_LEN - z] = 0;
		right_dma_Buffer[DMABUF_LEN - z ] = 0; // Extra time for latch (50us?)
	}
	uint32_t PRIMASK_STATE =  __get_PRIMASK();// Store the Current State of the Interrupts in (PRIMASK)
//	// Note: CPSID i sets PRIMASK to 1 (disable) and CPSIE i clears PRIMASK to 0 (enables)
	__disable_irq(); // This is an Critical Section because we want to Ensure both DMA Start without interrupts in between
	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, (uint32_t*)right_dma_Buffer, DMABUF_LEN);
	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t*)left_dma_Buffer, DMABUF_LEN);
	__set_PRIMASK(PRIMASK_STATE);
}

void updateDash(uint8_t blinkData_local){
	if (blinkData_local == prev_blinkData) { // Case where toggle hazard, it will just return
		return;
	}
	datasentFlag = 0; // Prevent BlinkData from being overwrite while executing
	// Blink + Left/Right can be On Concurrently

	if(blinkData_local & 0b1000){
		for(int i = 0; i < CUT_OFF; i++){
			SetPixelColor(&Left_PixelData[i], HEADLIGHT_COLOR);
			SetPixelColor(&Right_PixelData[i], HEADLIGHT_COLOR);
		}
	}else{
		for(int i = 0; i < CUT_OFF; i++){
			SetPixelColor(&Left_PixelData[i], OFF_COLOR);
			SetPixelColor(&Right_PixelData[i], OFF_COLOR);
		}
	}

	if ((~blinkData_local & 0b0010) && (blinkData_local&0b0001)){ // If Left is off and Right Is on, ensure left blink is off
		for(int i = CUT_OFF; i < NUM_PIXEL; i++){
			SetPixelColor(&Left_PixelData[i], OFF_COLOR);
		}
		LEFT_BLINK = 0;
		LEFT_BLINK_FLAG = 0;
		RIGHT_BLINK = 1;
		RIGHT_BLINK_FLAG = 1;
		counter =  0;

	}
	else if((~blinkData_local & 0b0001) && (blinkData_local & 0b0010)){ // If Right is off and Left is On, ensure right blink is off

		for(int i = CUT_OFF; i < NUM_PIXEL; i++){
			SetPixelColor(&Right_PixelData[i], OFF_COLOR);
		}
		RIGHT_BLINK = 0;
		RIGHT_BLINK_FLAG = 0;
		LEFT_BLINK = 1;
		LEFT_BLINK_FLAG = 1;
		counter = 0;
	}

	else{ // Toggled all Off
//		 No Blinking
		for(int i = CUT_OFF; i < NUM_PIXEL; i++){
				SetPixelColor(&Right_PixelData[i], OFF_COLOR);
			}
		for(int j = CUT_OFF; j < NUM_PIXEL; j++){
			SetPixelColor(&Left_PixelData[j], OFF_COLOR);
		}
		LEFT_BLINK = 0;
		LEFT_BLINK_FLAG = 1; // Reset it to default
		RIGHT_BLINK = 0;
		RIGHT_BLINK_FLAG = 1; // Reset it to default
		counter =  INT_MAX - 800;

	}
	prev_blinkData = blinkData_local;
	updateLight();
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
   HAL_GPIO_WritePin(LVL_SHIFT_EN_GPIO_Port, LVL_SHIFT_EN_Pin, GPIO_PIN_SET);
   HAL_GPIO_WritePin(MUX_SEL_GPIO_Port, MUX_SEL_Pin, GPIO_PIN_RESET); // 0 Select LED_A_5V to LED_A_OUTPUT rather than LED_B_5V
   datasentFlag = 0b00000011;
   updateDashFlag = 0;

   LEFT_BLINK = 0;
   LEFT_BLINK_FLAG = 1;
   RIGHT_BLINK = 0;
   RIGHT_BLINK_FLAG = 1;


   blinkData = 0b0000;
   prev_blinkData = 0b0000;
   LightsInit();

   HAL_CAN_Start(&hcan);
   // Triggers Interrupt whenever FIFO0 receive a new message
   HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_WritePin(LVL_SHIFT_EN_GPIO_Port, LVL_SHIFT_EN_Pin, GPIO_PIN_SET); // Enable PWM Conversion Step Up from 3.3V to 5V
	  HAL_GPIO_WritePin(MUX_SEL_GPIO_Port, MUX_SEL_Pin, GPIO_PIN_RESET); // 0 Select LED_A_5V to LED_A_OUTPUT rather than LED_B_5V
	  if(counter + 800 <= HAL_GetTick()){
		  if(LEFT_BLINK){
			  if(LEFT_BLINK_FLAG){
				  for(int i = CUT_OFF; i < NUM_PIXEL; i++){
					  SetPixelColor(&Left_PixelData[i], TURNSIG_COLOR);
				  }
				  LEFT_BLINK_FLAG = 0;
			  }
			  else if(LEFT_BLINK_FLAG == 0){
				  for(int i = CUT_OFF; i < NUM_PIXEL; i++){
					  SetPixelColor(&Left_PixelData[i], OFF_COLOR);
				  }
				  LEFT_BLINK_FLAG = 1;
			  }
		  }
		  if(RIGHT_BLINK){
			  if(RIGHT_BLINK_FLAG){
				  for(int i = CUT_OFF; i < NUM_PIXEL; i++){
					  SetPixelColor(&Right_PixelData[i], TURNSIG_COLOR);
				  }
				  RIGHT_BLINK_FLAG = 0 ;
			  }
			  else if(RIGHT_BLINK_FLAG == 0){

				  for(int i = CUT_OFF; i < NUM_PIXEL; i++){
					  SetPixelColor(&Right_PixelData[i], OFF_COLOR);
				  }
				  RIGHT_BLINK_FLAG = 1;
			  }
		  }
		  counter = HAL_GetTick();
		  updateLight();
	  }

	  // Interrupt won't be faster than CPU execution since processing speed is way faster
	  if(updateDashFlag && datasentFlag == 0b0011){
		  updateDash(blinkData);
		  updateDashFlag = 0 ; // 0 means the latest dash data has been processed

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
  canfilterconfig.FilterIdLow = 0xFFFF;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
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
  HAL_GPIO_WritePin(MUX_SEL_GPIO_Port, MUX_SEL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LVL_SHIFT_EN_GPIO_Port, LVL_SHIFT_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MUX_SEL_Pin */
  GPIO_InitStruct.Pin = MUX_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MUX_SEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LVL_SHIFT_EN_Pin */
  GPIO_InitStruct.Pin = LVL_SHIFT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LVL_SHIFT_EN_GPIO_Port, &GPIO_InitStruct);

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
