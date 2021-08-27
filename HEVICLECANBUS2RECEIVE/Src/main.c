/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd16x2.h"
#include "DC_MOTOR.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define usTIM	TIM4
#define DC_MOTOR1    0
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint16_t state_mode = 0;
HAL_StatusTypeDef ret;
uint16_t master_ID = 0x123;
uint16_t slave_ID = 0x124;
float Distance = 0;

uint8_t value = 50;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Received_Value (uint16_t temp_val);
void Delay_us(uint32_t delay);
float readdistance(void);
int bus_error = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t             TxData[8];
uint8_t             RxData[8];
uint32_t            TxMailbox;

int check_data = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{


    if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
    	Error_Handler();
    	lcd16x2_1stLine();
        lcd16x2_printf("CAN BUS ERROR");
        HAL_Delay(1000);
    }


    if(RxHeader.StdId == master_ID)

    {
    	if(RxData[0] > 30)
    	{
    		check_data = 1;
    		state_mode = 1;   //prints temp

    		if(RxData[1] < 50)
    		{
    			check_data = 1;
    			state_mode = 2;  // prints temp and pressure

    			if(RxData[2] < 10)
    		    {
    		    	check_data = 1;
    		    	state_mode = 3;   //prints temp pressure and distance
    		    }
    		}
    		else if(RxData[2] < 10)
    		{
    			check_data = 1;
    			state_mode = 7;
    		}
    	 }
    	 else if (RxData[1] < 50)
    	 {
    		check_data = 1;
    		state_mode = 4;  //prints pressure only
    		if(RxData[0] > 30)
    		{
    			check_data = 1;
    			state_mode = 2;  //prints temp and pressure

    			if(RxData[2] < 10)
    	    	{
    	    	    check_data = 1;
    	    		state_mode = 3;  // prints pressure and distance
    	    	}
    		}
    		else if(RxData[2] < 10)
    		{
    			check_data = 1;
    			state_mode = 5;  // prints pressure and distance
    		}
    	 }
    	 else if(RxData[2] < 10)
    	 {
    		check_data = 1;
    		state_mode = 6;   // prints distance
    		if(RxData[0] > 30)
    		{
    			check_data = 1;
    			state_mode = 7; // prints temp and distance

    			if(RxData[1] < 50)
    			{
    			    check_data = 1;
    			    state_mode = 3; // prints temp pressure distance
    			}
    		}
    		else if(RxData[1] < 50)
    		{
    		    check_data = 1;
    		    state_mode = 7; // prints temp pressure distance
    		}
    	 }
    	 else
    	 {
    		check_data = 1;
    		state_mode = 0;
    	 }

    }

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//uint16_t AD_RES = 0;
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
  MX_CAN1_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    /* Start Error */
     Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {

    /* Notification Error */
    Error_Handler();
  }

  //TxHeader.StdId = master_ID;
  TxHeader.StdId = slave_ID;
  TxHeader.ExtId = 0x00;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 4;
  TxHeader.TransmitGlobalTime = DISABLE;


  /* LCD Pins */
    lcd16x2_init_8bits(RS_GPIO_Port, RS_Pin, E_Pin,
     D0_GPIO_Port, D0_Pin, D1_Pin, D2_Pin, D3_Pin,
     D4_GPIO_Port, D4_Pin, D5_Pin, D6_Pin, D7_Pin);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(check_data == 1)
	  {
		  Received_Value(state_mode);
	  }
	  //htim3.Instance->CCR1 = value;
	  // Start ADC Conversion
	     //HAL_ADC_Start(&hadc1);
	 // Poll ADC1 Perihperal & TimeOut = 1mSec
	 	 //HAL_ADC_PollForConversion(&hadc1, 1);
	 // Read The ADC Conversion Result & Map It To PWM DutyCycle
	 	 //AD_RES = HAL_ADC_GetValue(&hadc1);
	 	 //DC_MOTOR_Set_Speed(DC_MOTOR1, AD_RES>>2);
	 	 //HAL_Delay(1);

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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  hcan1.Init.Prescaler = 14;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
	bus_error = 1;
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  /* CAN filter */
   sFilterConfig.FilterBank = 18;
   sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
   sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
   sFilterConfig.FilterIdHigh = 0x0000;
   sFilterConfig.FilterIdLow = 0x0000;
   sFilterConfig.FilterMaskIdHigh = 0x0000;
   sFilterConfig.FilterMaskIdLow = 0x0000;
   sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
   sFilterConfig.FilterActivation = ENABLE;
   sFilterConfig.SlaveStartFilterBank = 20;
   if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
   {
      /* Filter configuration Error */
       Error_Handler();
   }

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
//
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
//
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
//
  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 110-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 168-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|D0_Pin|D1_Pin|D2_Pin
                          |D3_Pin|D4_Pin|D5_Pin|D6_Pin
                          |D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RS_Pin|E_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 D0_Pin D1_Pin D2_Pin
                           D3_Pin D4_Pin D5_Pin D6_Pin
                           D7_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_12|D0_Pin|D1_Pin|D2_Pin
                          |D3_Pin|D4_Pin|D5_Pin|D6_Pin
                          |D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO_Pin */
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RS_Pin E_Pin */
  GPIO_InitStruct.Pin = RS_Pin|E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Received_Value (uint16_t mode_state)
{

	switch(mode_state)
		      {
		          case(0): //IDE
		          {
		        	  lcd16x2_clear();
		        	  HAL_Delay(500);
		        	  lcd16x2_1stLine();
		        	  lcd16x2_printf("Waiting for CAN");
		        	  HAL_Delay(1000);

		          }
		          break;
		          case(1):
	              {
		        	  lcd16x2_clear();
		        	  HAL_Delay(500);
		        	  lcd16x2_1stLine();
		        	  lcd16x2_printf("Temp high : %.2f",(float) RxData[0]);
		        	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		        	  htim3.Instance->CCR1 = value;
		        	  HAL_Delay(1000);

		        	  value = 0;
	              }
		          break;
		          case(2):
	              {
		        	  lcd16x2_clear();
		        	  HAL_Delay(500);
		        	  lcd16x2_1stLine();
		        	  lcd16x2_printf("Temp high : %.2f",(float) RxData[0]);
		        	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		        	  htim3.Instance->CCR1 = value;
		        	  HAL_Delay(1000);
		        	  value = 0;

		        	  lcd16x2_clear();
		        	  HAL_Delay(500);
		        	  lcd16x2_1stLine();
		        	  lcd16x2_printf("Pressure: %.2f",(float) RxData[1]);
		        	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		        	  //htim3.Instance->CCR1 = value;
		        	  HAL_Delay(1000);

	              }
		          break;
		          case(3):
	              {
		        	  lcd16x2_clear();
		        	  HAL_Delay(500);
		        	  lcd16x2_1stLine();
		        	  lcd16x2_printf("Temp high : %.2f",(float) RxData[0]);
		        	  htim3.Instance->CCR1 = value;
		        	  HAL_Delay(1000);
		        	  value = 0;

		        	  lcd16x2_clear();
		        	  HAL_Delay(500);
		        	  lcd16x2_1stLine();
		        	  lcd16x2_printf("Pressure: %.2f",(float) RxData[1]);
		        	 // htim3.Instance->CCR1 = value;
		        	  HAL_Delay(1000);

		        	  lcd16x2_clear();
		        	  HAL_Delay(500);
		        	  lcd16x2_1stLine();
		        	  lcd16x2_printf("Distance: %.1f",(float) RxData[2]);
		        	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		        	  //htim3.Instance->CCR1 = value;
		        	  HAL_Delay(1000);

	              }
		          break;
		          case(4):
	              {
		        	  lcd16x2_clear();
		        	  HAL_Delay(500);
		        	  lcd16x2_1stLine();
		        	  lcd16x2_printf("Pressure: %.2f",(float) RxData[1]);
		        	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		        	  htim3.Instance->CCR1 = value;
		        	  HAL_Delay(1000);

		        	  value = 0;
	              }
		          break;
		          case(5):
	              {
		        	  lcd16x2_clear();
		        	  HAL_Delay(500);
		        	  lcd16x2_1stLine();
		        	  lcd16x2_printf("Pressure: %.2f",(float) RxData[1]);
		        	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		        	  htim3.Instance->CCR1 = value;
		        	  HAL_Delay(1000);
		        	  value = 0;

		        	  lcd16x2_clear();
		        	  HAL_Delay(500);
		        	  lcd16x2_1stLine();
		        	  lcd16x2_printf("Distance: %.1f",(float) RxData[2]);
		        	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		        	 // htim3.Instance->CCR1 = value;
		        	  HAL_Delay(1000);

	              }
		          break;
		          case(6):
	              {
		        	  lcd16x2_clear();
		        	  HAL_Delay(500);
		        	  lcd16x2_1stLine();
		        	  lcd16x2_printf("Distance: %.1f",(float) RxData[2]);
		        	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		        	  htim3.Instance->CCR1 = value;
		        	  HAL_Delay(1000);

		        	  value = 0;
	              }
		          break;
		          case(7):
		          {
		        	  lcd16x2_clear();
		        	  HAL_Delay(500);
		        	  lcd16x2_1stLine();
		        	  lcd16x2_printf("Temp high : %.2f",(float) RxData[0]);
		        	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		        	  htim3.Instance->CCR1 = value;
		        	  HAL_Delay(1000);
		        	  value = 0;

		        	  lcd16x2_clear();
		        	  HAL_Delay(500);
		        	  lcd16x2_1stLine();
		        	  lcd16x2_printf("Distance: %.1f",(float) RxData[2]);
		        	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		        	 // htim3.Instance->CCR1 = value;
		        	  HAL_Delay(1000);

		          }
		          break;
		          default:
				  break;
		      }
}

float readdistance(void)
{
	const float speedOfSound = 0.0343/2;
	uint32_t numTicks = 0;
	float distance = 0;

	//Set TRIG to LOW for few uSec
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
	Delay_us(3);

	//*** START Ultrasonic measure routine ***//
	//Output 10 usec TRIG
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	Delay_us(10);
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

	//Wait for ECHO pin rising edge
	while(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET);

	//Start measuring ECHO pulse width in usec
	numTicks = 0;
	while(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET)
	{
		numTicks++;
	    //Delay_us(2); //2.8usec
	};

	//Estimate distance in cm
	distance = (numTicks + 0.0f) * speedOfSound;

	return(distance);
}


void Delay_us(uint32_t delay)
{
	if(delay < 2) delay = 2;
	usTIM->ARR = delay - 1; 	/*sets the value in the auto-reload register*/
	usTIM->EGR = 1; 			/*Re-initialises the timer*/
	usTIM->SR &= ~1; 		//Resets the flag
	usTIM->CR1 |= 1; 		//Enables the counter
	while((usTIM->SR&0x0001) != 1);
	usTIM->SR &= ~(0x0001);
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
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
