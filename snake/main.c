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
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  RIGHT,
  DOWN,
  LEFT,
  UP
}MOVEMENT;

typedef enum
{
  HIGH,
  LOW
}TONE;

typedef enum
{
  FOURTH,
  THIRD
}DIGITS;

struct snake
{
    struct bodypart * head;
	struct bodypart * tail;
	struct bodypart * target;
    uint8_t length;
};

struct bodypart
{
    int8_t x, y;
    struct bodypart * prev;
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ALL_COLS			(C1_Pin|C2_Pin|C3_Pin|C4_Pin|C5_Pin|C6_Pin|C7_Pin|C8_Pin)
#define START_X				-1
#define START_Y				0
#define DIAL_SENSITIVITY	2
#define MIN_SPEED_PERIOD    50
#define BUZZ_LEN			24
#define BANG_TIM2_ARR		300000
#define RST_DISP_ADDR_PA	(A_LED_Pin|B_LED_Pin|C_LED_Pin|D_LED_Pin|E_LED_Pin)
#define RST_DISP_ADDR_PC	(F_LED_Pin|G_LED_Pin)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint8_t movement = RIGHT;
bool bang = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
struct bodypart * createBodypart(struct snake * snPtr);
struct bodypart * createSnack(uint16_t * dispData, const uint16_t * Cols);
void elongate(struct snake * snPtr, uint16_t * dispData, const uint16_t * Cols);
void swap(struct snake * snPtr, uint16_t * dispData, const uint16_t * Cols);
void display(uint8_t value, uint8_t digit);
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
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  //HAL_GPIO_WritePin(N_ENABLE_MEASURE_GPIO_Port, N_ENABLE_MEASURE_Pin, GPIO_PIN_SET);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  float newPeriodTIM2 = HAL_ADC_GetValue(&hadc1);
  newPeriodTIM2 = ((2048/newPeriodTIM2-1)*DIAL_SENSITIVITY+1)*TIM2->ARR;
  TIM2->ARR = newPeriodTIM2>MIN_SPEED_PERIOD ? newPeriodTIM2:MIN_SPEED_PERIOD;
  //HAL_GPIO_WritePin(N_ENABLE_MEASURE_GPIO_Port, N_ENABLE_MEASURE_Pin, GPIO_PIN_RESET);

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_TIM2
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 499999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  htim4.Init.Prescaler = 72;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, B_Pin|A_Pin|D4_LED_Pin|D3_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, C4_Pin|C2_Pin|C5_Pin|C7_Pin
                          |C1_Pin|C3_Pin|C6_Pin|C8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, R2_Pin|R6_Pin|R1_Pin|R7_Pin
                          |R8_Pin|R3_Pin|R5_Pin|R4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, G_LED_Pin|F_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, E_LED_Pin|D_LED_Pin|C_LED_Pin|B_LED_Pin
                          |A_LED_Pin|BUZZER_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D_Pin C_Pin */
  GPIO_InitStruct.Pin = D_Pin|C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : B_Pin A_Pin */
  GPIO_InitStruct.Pin = B_Pin|A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin E_LED_Pin D_LED_Pin C_LED_Pin
                           B_LED_Pin A_LED_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|E_LED_Pin|D_LED_Pin|C_LED_Pin
                          |B_LED_Pin|A_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : C4_Pin C2_Pin R2_Pin R6_Pin
                           C5_Pin R1_Pin C7_Pin R7_Pin
                           R8_Pin R3_Pin C1_Pin R5_Pin
                           C3_Pin C6_Pin C8_Pin R4_Pin */
  GPIO_InitStruct.Pin = C4_Pin|C2_Pin|R2_Pin|R6_Pin
                          |C5_Pin|R1_Pin|C7_Pin|R7_Pin
                          |R8_Pin|R3_Pin|C1_Pin|R5_Pin
                          |C3_Pin|C6_Pin|C8_Pin|R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : D4_LED_Pin D3_LED_Pin G_LED_Pin F_LED_Pin */
  GPIO_InitStruct.Pin = D4_LED_Pin|D3_LED_Pin|G_LED_Pin|F_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static const uint16_t Cols[8] = {C1_Pin,C2_Pin,C3_Pin,C4_Pin,C5_Pin,C6_Pin,C7_Pin,C8_Pin};
	static const uint16_t Rows[8] = {R1_Pin|ALL_COLS,R2_Pin|ALL_COLS,R3_Pin|ALL_COLS,R4_Pin|ALL_COLS,
									R5_Pin|ALL_COLS,R6_Pin|ALL_COLS,R7_Pin|ALL_COLS,R8_Pin|ALL_COLS};
	static uint16_t dispData[8] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
	static bool toggleTest;
	static uint8_t buzzerCt;
	static uint8_t tone = HIGH;

	// TODO createSnake()
	static struct bodypart head = {.x = START_X, .y = START_Y, .prev = NULL};
	static struct bodypart tail = {.x = START_X-1, .y = START_Y, .prev = &head};
	static struct snake sn = {.head = &head, .tail = &tail, .target = NULL, .length = 2};

	if(htim->Instance==TIM4){
		static uint8_t pos = 0;

		if(bang){
			TIM2->ARR = BANG_TIM2_ARR;
			static const uint16_t TestX[8] = {C1_Pin|C8_Pin, C2_Pin|C7_Pin, C3_Pin|C6_Pin, C4_Pin|C5_Pin,
											C5_Pin|C4_Pin, C6_Pin|C3_Pin, C7_Pin|C2_Pin, C8_Pin|C1_Pin};
			static const uint16_t TestO[8] = {C3_Pin|C4_Pin|C5_Pin|C6_Pin, C2_Pin|C7_Pin, C1_Pin|C8_Pin,
											C1_Pin|C8_Pin, C1_Pin|C8_Pin, C1_Pin|C8_Pin,
											C2_Pin|C7_Pin, C3_Pin|C4_Pin|C5_Pin|C6_Pin};

			GPIOB->ODR = Rows[pos]^~(toggleTest ? TestX[pos]:TestO[pos]);

			if(buzzerCt<6*BUZZ_LEN){
				switch(tone){
				case HIGH:
					if(!(pos%4)) HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
					break;
				case LOW:
					if(!pos) HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
					break;
				}
				buzzerCt++;
			}
		}
		else if(sn.length==64){
			TIM2->ARR = BANG_TIM2_ARR;
			static const uint16_t Trophy[8] = {ALL_COLS,
											ALL_COLS,
											C2_Pin|C3_Pin|C4_Pin|C5_Pin|C6_Pin|C7_Pin,
											C3_Pin|C4_Pin|C5_Pin|C6_Pin,
											C4_Pin|C5_Pin,
											C4_Pin|C5_Pin,
											C3_Pin|C4_Pin|C5_Pin|C6_Pin,
											C2_Pin|C3_Pin|C4_Pin|C5_Pin|C6_Pin|C7_Pin};

			GPIOB->ODR = Rows[pos]^~Trophy[pos];

			if(buzzerCt<BUZZ_LEN){
				switch(tone){
				case HIGH:
					if(!(pos%2)) HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
					break;
				case LOW:
					HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
					break;
				}
				buzzerCt++;
			}
			else if(tone) HAL_TIM_Base_Stop_IT(&htim2);
		}
		else{
			GPIOB->ODR = Rows[pos]^~dispData[pos];
			if(buzzerCt<BUZZ_LEN){
				HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
				buzzerCt++;
			}
		}

		if(!(pos%4)) display(sn.length-1, pos/4);
		pos = (pos+1)%8;
	}
	if(htim->Instance==TIM3){
		static bool togglePin;
		togglePin = !togglePin;
		if(togglePin){
			HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_SET);
		}
	}
	else if(htim->Instance==TIM2){
		toggleTest = !toggleTest;
		buzzerCt = 0;
		if(!sn.target) sn.target = createSnack(dispData, Cols);
		if(!sn.target) bang = true;
		if(bang || sn.length==64){
			static uint8_t blinks = 0;
			if(blinks<5){
				if(!tone) blinks++;
				tone = (tone+1)%2;
			}
			else tone = 2;
		}
		else{
			switch(movement){
				case DOWN:
					if(sn.head->y==7) bang = true;
					else if(sn.head->x==sn.target->x && sn.head->y+1==sn.target->y){
						elongate(&sn, dispData, Cols);
					}
					else if(dispData[sn.head->y+1]!=(dispData[sn.head->y+1]&~Cols[sn.head->x])) bang = true;
					else{
						if(sn.tail->x>=0) dispData[sn.tail->y] ^= Cols[sn.tail->x]; // usuń ogon z wyświetlania – zeruj bity
						sn.tail->x = sn.head->x;
						sn.tail->y = sn.head->y+1;
						swap(&sn, dispData, Cols);
					}
					break;
				case RIGHT:
					if(sn.head->x==7) bang = true;
					else if(sn.head->x+1==sn.target->x && sn.head->y==sn.target->y){
						elongate(&sn, dispData, Cols);
					}
					else if(dispData[sn.head->y]!=(dispData[sn.head->y]&~Cols[sn.head->x+1])) bang = true;
					else{
						if(sn.tail->x>=0) dispData[sn.tail->y] ^= Cols[sn.tail->x];
						sn.tail->x = sn.head->x+1;
						sn.tail->y = sn.head->y;
						swap(&sn, dispData, Cols);
					}
					break;
				case UP:
					if(sn.head->y==0) bang = true;
					else if(sn.head->x==sn.target->x && sn.head->y-1==sn.target->y){
						elongate(&sn, dispData, Cols);
					}
					else if(dispData[sn.head->y-1]!=(dispData[sn.head->y-1]&~Cols[sn.head->x])) bang = true;
					else{
						if(sn.tail->x>=0) dispData[sn.tail->y] ^= Cols[sn.tail->x];
						sn.tail->x = sn.head->x;
						sn.tail->y = sn.head->y-1;
						swap(&sn, dispData, Cols);
					}
					break;
				case LEFT:
					if(sn.head->x==0) bang = true;
					else if(sn.head->x-1==sn.target->x && sn.head->y==sn.target->y){
						elongate(&sn, dispData, Cols);
					}
					else if(dispData[sn.head->y]!=(dispData[sn.head->y]&~Cols[sn.head->x-1])) bang = true;
					else{
						if(sn.tail->x>=0) dispData[sn.tail->y] ^= Cols[sn.tail->x];
						sn.tail->x = sn.head->x-1;
						sn.tail->y = sn.head->y;
						swap(&sn, dispData, Cols);
					}
					break;
			}
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static GPIO_PinState a, b;
	a = HAL_GPIO_ReadPin(A_GPIO_Port, A_Pin);
	b = HAL_GPIO_ReadPin(B_GPIO_Port, B_Pin);
	if(GPIO_Pin==B1_Pin){
		if(TIM2->CR1&1){
			HAL_TIM_Base_Stop_IT(&htim2);
			HAL_TIM_Base_Stop_IT(&htim3);
		}
		else{
			HAL_TIM_Base_Start_IT(&htim2);
			HAL_TIM_Base_Start_IT(&htim3);
		}
	}
	else if(GPIO_Pin==C_Pin){
		if(a) movement = UP;
		else if(b) movement = LEFT;
	}
	else if(GPIO_Pin==D_Pin){
		if(a) movement = DOWN;
		else if(b) movement = RIGHT;
	}
}

struct bodypart * createBodypart(struct snake * snPtr){
	snPtr->length++;
	struct bodypart * bpPtr = malloc(sizeof(struct bodypart));
	if(bpPtr==NULL) return NULL;
	struct bodypart bp = {.x = snPtr->target->x, .y = snPtr->target->y, .prev = NULL};
	*bpPtr = bp;
	return bpPtr;
}

struct bodypart * createSnack(uint16_t * dispData, const uint16_t * Cols){
	uint8_t x, y;
	for(y = rand()%8;;y = (y+1)%8){ // infinite loop
		if(dispData[y]!=ALL_COLS){
			for(x = rand()%8;;x = (x+1)%8){ // infinite loop
				if(dispData[y]!=(dispData[y]|Cols[x])){
					dispData[y] = dispData[y]|Cols[x];
					struct bodypart * targetPtr = malloc(sizeof(struct bodypart));
					if(targetPtr==NULL) return NULL;
					struct bodypart target = {.x = x, .y = y, .prev = NULL};
					*targetPtr = target;
					return targetPtr;
				}
			}
		}
	}
}

void elongate(struct snake * snPtr, uint16_t * dispData, const uint16_t * Cols){
	snPtr->head->prev = createBodypart(snPtr);
	snPtr->head = snPtr->head->prev;
	free(snPtr->target);
	snPtr->target = createSnack(dispData, Cols);
}

void swap(struct snake * snPtr, uint16_t * dispData, const uint16_t * Cols){
	snPtr->head->prev = snPtr->tail;
	snPtr->head = snPtr->tail;
	snPtr->tail = snPtr->tail->prev;
	snPtr->head->prev = NULL;
	if(snPtr->head->x>=0) dispData[snPtr->head->y] ^= Cols[snPtr->head->x]; // dodaj głowę do wyświetlania – ustaw bity
}

void display(uint8_t value, uint8_t digit){
	static uint8_t buffer;
	static const uint16_t signsPA[10] = {A_LED_Pin|B_LED_Pin|C_LED_Pin|D_LED_Pin|E_LED_Pin,
										B_LED_Pin|C_LED_Pin,
										A_LED_Pin|B_LED_Pin|D_LED_Pin|E_LED_Pin,
										A_LED_Pin|B_LED_Pin|C_LED_Pin|D_LED_Pin,
										B_LED_Pin|C_LED_Pin,
										A_LED_Pin|C_LED_Pin|D_LED_Pin,
										A_LED_Pin|C_LED_Pin|D_LED_Pin|E_LED_Pin,
										A_LED_Pin|B_LED_Pin|C_LED_Pin,
										A_LED_Pin|B_LED_Pin|C_LED_Pin|D_LED_Pin|E_LED_Pin,
										A_LED_Pin|B_LED_Pin|C_LED_Pin|D_LED_Pin};
	static const uint16_t signsPC[10] = {F_LED_Pin,
										0,
										G_LED_Pin,
										G_LED_Pin,
										F_LED_Pin|G_LED_Pin,
										F_LED_Pin|G_LED_Pin,
										F_LED_Pin|G_LED_Pin,
										0,
										F_LED_Pin|G_LED_Pin,
										F_LED_Pin|G_LED_Pin,};

	HAL_GPIO_WritePin(GPIOA,RST_DISP_ADDR_PA,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,RST_DISP_ADDR_PC,GPIO_PIN_SET);
	switch(digit){
	case THIRD:
		buffer = value/10 ? value/10:10;
		HAL_GPIO_WritePin(GPIOC,D4_LED_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,D3_LED_Pin,GPIO_PIN_SET);
		break;
	case FOURTH:
		buffer = 10!=buffer ? value-10*buffer:value;
		HAL_GPIO_WritePin(GPIOC,D3_LED_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,D4_LED_Pin,GPIO_PIN_SET);
		break;
	}
	if(buffer!=10){
		HAL_GPIO_WritePin(GPIOA,signsPA[buffer],GPIO_PIN_RESET);
		if(buffer!=1 && buffer!=7) HAL_GPIO_WritePin(GPIOC,signsPC[buffer],GPIO_PIN_RESET);
	}
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
