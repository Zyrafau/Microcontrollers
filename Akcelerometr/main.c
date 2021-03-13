/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CTR_REG1	0x20
#define OUT_X		0x29
#define OUT_Y		0x2B
#define OUT_Z		0x2D
#define GRAVITY		9.8 // [m/s**2]
#define DOT_POINT	0x0080
#define CLR_SCR		0xFFFF
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
SPI_HandleTypeDef hspi1;
const float coeff = 2*GRAVITY/255; // współczynnik kierunkowy prostej
const uint16_t Positions[4] = {0x0800,0x0400,0x0200,0x0100}; // [COM1,COM2,COM3,COM4]
const uint16_t Signs[13] = {0x0081,0x00E7,0x00A8,0x00A4,
				   0x00C6,0x0094,0x0090,
				   0x00A7,0x0080,0x0084,
				   0x00FE,0x00FF,0x0098}; // [0,…,9,minus,blankspace,Error]

float ax_val[3];
float disp_val;
//uint8_t buffer = 0;
uint8_t buffer = 12; // Error sign
uint8_t pos = 0 ;
uint8_t wybor_osi = 0;
uint8_t znak_uart, kom[50];
uint16_t dl_kom;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void transmit(uint8_t addr,uint8_t msg);
uint8_t receive(uint8_t addr);
void display(float val);
float read(uint8_t val){ return coeff*receive(val)-GRAVITY;}
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  __HAL_SPI_ENABLE(&hspi1);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_UART_Receive_IT(&huart2, &znak_uart, 1);

  transmit(CTR_REG1,0b01000111);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void transmit(uint8_t addr, uint8_t msg){
	uint8_t tx[] = {addr,msg};
	HAL_GPIO_WritePin(NSS_GPIO_Port,NSS_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,tx,2,HAL_MAX_DELAY);
	HAL_GPIO_WritePin(NSS_GPIO_Port,NSS_Pin,GPIO_PIN_SET);
}

uint8_t receive(uint8_t addr){
	uint8_t tx[] = {addr|=1<<7,0xFF};
	uint8_t rx[2];
	HAL_GPIO_WritePin(NSS_GPIO_Port,NSS_Pin,GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1,tx,rx,2,HAL_MAX_DELAY); // bo sam nie wyśle
	HAL_GPIO_WritePin(NSS_GPIO_Port,NSS_Pin,GPIO_PIN_SET);
	return rx[1]; // bo wysyła tyle, ile dostał, a nie chcemy addr, lecz msg
}

void display(float val){
	val = round(10*val);
	float abs = fabsf(val);

        switch(pos){
	   case 0:
		buffer = 0;
	  	GPIOC->ODR = ~(Positions[pos] | Signs[val<0 ? 10:11]);
		break;
           case 1:
		buffer = floor(abs/100);
		GPIOC->ODR = ~(Positions[pos] | Signs[buffer]);
		break;
           case 2:
		buffer = floor((abs-100*buffer)/10);
		GPIOC->ODR = ~(Positions[pos] | Signs[buffer]) ^ DOT_POINT;
		break;
           case 3:
		buffer = abs-10*floor(abs/10);
		GPIOC->ODR = ~(Positions[pos] | Signs[buffer]);
		break;
           default:
		buffer = 12;
		//GPIOC->ODR = CLR_SCR;
        }

        pos = pos<3 ? pos+1:0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM1){
		ax_val[0] = read(OUT_X);
		ax_val[1] = read(OUT_Y);
		ax_val[2] = read(OUT_Z);
		disp_val = ax_val[wybor_osi];
	}
	else if(htim->Instance==TIM3){ display(disp_val);}
	else if(htim->Instance==TIM10){
		dl_kom = sprintf(kom,"Dane:\r\nx: %.1f\r\ny: %.1f\r\nz: %.1f\r\n\r\n",
						ax_val[0],ax_val[1],ax_val[2]);
		HAL_UART_Transmit_IT(&huart2,kom,dl_kom);
		HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin==BLUE_BUTTON_Pin){ wybor_osi = wybor_osi<2 ? wybor_osi+1:0;}
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

  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
