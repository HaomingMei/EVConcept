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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_ACTIVATE_NOTIF_TIMEOUT 1000
#define DASHBOARD_CANID 0x0012
#define ACC_ID 3
#define TRANS_DEBOUNCE_LOOPS 50

//#define MAIN_CODE // Define if running the actual program, comment out the rest

// Below are functions created for unit testing
	// Uncomment them as necessary

#define UNIT_PWM_TEST
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
uint8_t RxMessage[8];
volatile uint8_t speed = 0; // No optimization
CAN_RxHeaderTypeDef RxHeader;
uint8_t updateFlag;
uint32_t tick;
uint8_t REST_DRIVE_COUNT;
uint8_t DRIVE_DECAY_COUNT;
uint8_t DECAY_DRIVE_COUNT;
uint8_t DECAY_REST_COUNT;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef MAIN_CODE
typedef enum{
	REST,
	DRIVE,
	SLOW_DECAY
}CAR_STATE;

volatile CAR_STATE current_state; // no optimization

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *can){
	if(HAL_CAN_GetRxMessage(can,CAN_FILTER_FIFO0, &RxHeader, RxMessage ) == HAL_OK){

		HAL_GPIO_TogglePin(DBGLED1_GPIO_Port, DBGLED1_Pin);
		if(RxHeader.StdId == ACC_ID){
			speed = (uint8_t)RxMessage[0];

		}
	}


}
void update_car_state(){
	switch(current_state){
		case REST:
			// High Side Siwtch is Off, Low Side Switch is Off
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			TIM16->CCR1 = 0;

			if(speed>0){
				REST_DRIVE_COUNT += 1;
			}
			else{
				REST_DRIVE_COUNT = 0;
			}
			if(REST_DRIVE_COUNT > TRANS_DEBOUNCE_LOOPS){
				current_state = DRIVE;
				REST_DRIVE_COUNT = 0;
			}
			break;

		case DRIVE:

			// High Side Siwtch is On, Low Side Switch is On
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // R2
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // R1

			TIM16->CCR1 = (uint32_t)(((uint32_t)speed * TIM16->ARR * 8) / (25 * 10));
			// 80% is the max duty cycle, speed ranges from 0-25

			if(speed == 0){
				DRIVE_DECAY_COUNT += 1;
			}
			else{
				DRIVE_DECAY_COUNT = 0;
			}
			if(DRIVE_DECAY_COUNT > TRANS_DEBOUNCE_LOOPS){
				current_state = SLOW_DECAY;
				DRIVE_DECAY_COUNT = 0;
			}
			break;

		case SLOW_DECAY:
			// High Side Siwtch is Off, Low Side Switch is On
			TIM16->CCR1 = 0;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // R1
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // R2
			if(speed == 0 ){
				DECAY_DRIVE_COUNT = 0;
				DECAY_REST_COUNT += 1;
			}
			else{
				DECAY_REST_COUNT = 0;
				DECAY_DRIVE_COUNT += 1;
			}
			if(DECAY_DRIVE_COUNT > TRANS_DEBOUNCE_LOOPS){
				DECAY_REST_COUNT = 0;
				DECAY_DRIVE_COUNT = 0;
				current_state = DRIVE;
			}
			else if(DECAY_REST_COUNT > TRANS_DEBOUNCE_LOOPS){
				DECAY_REST_COUNT = 0;
				DECAY_DRIVE_COUNT = 0;
				current_state = REST;

			}
			break;
		default:
			TIM16->CCR1 = 0;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // R1
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // R2
			current_state = REST;
			DECAY_REST_COUNT = DECAY_DRIVE_COUNT = DRIVE_DECAY_COUNT = REST_DRIVE_COUNT = 0;
			break;
	}
}
#endif
#ifdef UNIT_PWM_TEST
// This function tests the PMW output for Speed 0 to 25, which corresponds to 0 to 100% Duty Cycle
void PWM_UNIT_TEST(){ 	// Use Debug Mode and make sure the PWM looks right, then right it in real time
//	int counter = 0 ;
	// Ensures that the PWM hardware is on
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);

	//** Going from Speed = 0 to 25 BEGIN **
	for(int speedtest = 0; speedtest <26; speedtest++){

		TIM16->CCR1 = (uint32_t)(((uint32_t)speedtest * TIM16->ARR * 8) / (25 * 10));
	}
	// Skipping by 2
	for(int speedtest = 0; speedtest <26; speedtest++){

		TIM16->CCR1 = (uint32_t)(((uint32_t)speedtest * TIM16->ARR * 8) / (25 * 10));
		speedtest+=1;
	}
	// Skipping by 5
	for(int speedtest = 0; speedtest <26; speedtest++){
		TIM16->CCR1 = (uint32_t)(((uint32_t)speedtest * TIM16->ARR * 8) / (25 * 10));
		speedtest+=4;
	}

	// Skipping by 10
	for(int speedtest = 0; speedtest <26; speedtest++){
		TIM16->CCR1 = (uint32_t)(((uint32_t)speedtest * TIM16->ARR * 8) / (25 * 10));
		speedtest+=9;
	}
	// Speed 0 Spike to Speed 25
	for(int speedtest = 0; speedtest <26; speedtest++){
		TIM16->CCR1 = (uint32_t)(((uint32_t)speedtest * TIM16->ARR * 8) / (25 * 10));
		speedtest+=24;
	}

	//** END**

	// Going from Speed = 25 to 0



}


#endif

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
  MX_CAN_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
	#ifdef MAIN_CODE
  updateFlag = 0;
  REST_DRIVE_COUNT = 0;
  DRIVE_DECAY_COUNT = 0;
  DECAY_DRIVE_COUNT = 0;
  DECAY_REST_COUNT = 0;
  current_state = REST ;
	#endif

  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_ACTIVATE_NOTIF_TIMEOUT);
  // Activate the PWM Hardware Here

  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);

  tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	#ifdef MAIN_CODE
	uint32_t PRIMASK_STATE =  __get_PRIMASK();// Store the Current State of the Interrupts in (PRIMASK)
//	// Note: CPSID i sets PRIMASK to 1 (disable) and CPSIE i clears PRIMASK to 0 (enables)
	__disable_irq(); // This is an Critical Section because we want to Update the Relay  without disruption
	if(HAL_GetTick() - tick >= 10){ // 10ms * 50 loops = 500ms (min) between each state transition
		update_car_state();
		tick = HAL_GetTick();
	}

	__set_PRIMASK(PRIMASK_STATE);
//	__enable_irq();
	#endif

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 32;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
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
	canfilterconfig.FilterIdHigh = ACC_ID << 5;
	canfilterconfig.FilterIdLow = 0xFFFF;
	canfilterconfig.FilterMaskIdHigh = 0xFFFF;
	canfilterconfig.FilterMaskIdLow = 0xFFFF;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDLIST;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_16BIT;
	canfilterconfig.SlaveStartFilterBank = 0;

   HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 2399;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, R1_Pin|R2_Pin|DBGLED1_Pin|DGBLED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : R1_Pin R2_Pin DBGLED1_Pin DGBLED2_Pin */
  GPIO_InitStruct.Pin = R1_Pin|R2_Pin|DBGLED1_Pin|DGBLED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
