/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "lcd16x2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RESPONSE_TIMEOUT 4000
#define RX_BUFFER_SIZE 256
#define MODBUS_GENERATOR 0xA001
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t Rx1Buffer[RX_BUFFER_SIZE];
uint16_t Rx1Count = 0;

uint32_t requestStartTime = 0;
uint8_t isReceiving = 0;
//uint32_t LCDtime = 0;
//uint8_t LCDShow = 0;

uint8_t Rx2Buffer[RX_BUFFER_SIZE];
uint16_t Rx2Count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t ModbusCalcCRC(uint8_t *Frame, uint16_t LenFrame)
{
	uint16_t CntByte;
	uint16_t j;
	uint8_t bitVal;
	uint16_t CRCcalculate = 0xFFFF;
	for(CntByte=0;CntByte<LenFrame;CntByte++)
	{
		CRCcalculate ^= Frame[CntByte];
		for(j=0;j<8;j++)
		{
			bitVal = CRCcalculate & 0x0001;
			CRCcalculate = CRCcalculate >> 1;
			if(bitVal == 1)
				CRCcalculate ^= MODBUS_GENERATOR;
		}
	}
	return CRCcalculate;
}

void ResetBuffers(void)
{
	Rx1Count = 0;
	Rx2Count = 0;
	isReceiving = 0;
	memset(Rx1Buffer, 0, RX_BUFFER_SIZE);
	memset(Rx2Buffer, 0, RX_BUFFER_SIZE);
	requestStartTime = 0;
}

void CheckTimeOut(void)
{
	if (HAL_GetTick() - requestStartTime >= RESPONSE_TIMEOUT)
	{
		ResetBuffers();
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, Rx1Buffer, RX_BUFFER_SIZE);
		HAL_UARTEx_ReceiveToIdle_IT(&huart2, Rx2Buffer, RX_BUFFER_SIZE);
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart == &huart1)
	{
		Rx1Count = Size;
		requestStartTime = HAL_GetTick();
		isReceiving = 1;
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, Rx1Buffer, RX_BUFFER_SIZE);
		HAL_UARTEx_ReceiveToIdle_IT(&huart2, Rx2Buffer, RX_BUFFER_SIZE);
	}
	if (huart == &huart2)
	{
		Rx2Count = Size;
		Rx1Count = 0;
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, Rx1Buffer, RX_BUFFER_SIZE);
		HAL_UARTEx_ReceiveToIdle_IT(&huart2, Rx2Buffer, RX_BUFFER_SIZE);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	lcd16x2_init_4bits(RS_GPIO_Port, RS_Pin, E_Pin, GPIOB, D4_Pin, D5_Pin, D6_Pin, D7_Pin);
	lcd16x2_clear();
	HAL_GPIO_WritePin(DE_RE_GPIO_Port, DE_RE_Pin, GPIO_PIN_RESET);
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, Rx1Buffer, RX_BUFFER_SIZE);
	HAL_UARTEx_ReceiveToIdle_IT(&huart2, Rx2Buffer, RX_BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		//lcd16x2_1stLine();
		//lcd16x2_printf("Num of M: %u", Rx1Count);
		//lcd16x2_2ndLine();
		//lcd16x2_printf("Num of S: %u", Rx2Count);
		if (Rx1Count > 5)
		{
			/*HAL_GPIO_WritePin(DE_RE_GPIO_Port, DE_RE_Pin, GPIO_PIN_SET);
			HAL_Delay(20);
			HAL_UART_Transmit(&huart2, Rx1Buffer, Rx1Count, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(DE_RE_GPIO_Port, DE_RE_Pin, GPIO_PIN_RESET);
			Rx1Count = 0;*/
			lcd16x2_1stLine();
		  lcd16x2_printf("Num of M: %u", Rx1Count);
			
			uint16_t RequestCRC;
			RequestCRC = (Rx1Buffer[Rx1Count - 1] << 8) | Rx1Buffer[Rx1Count - 2];
			uint16_t CRCCalc = ModbusCalcCRC(Rx1Buffer, Rx1Count - 2);
			if (RequestCRC == CRCCalc)
			{
				HAL_GPIO_WritePin(DE_RE_GPIO_Port, DE_RE_Pin, GPIO_PIN_SET);
				HAL_Delay(20);
				HAL_UART_Transmit(&huart2, Rx1Buffer, Rx1Count, HAL_MAX_DELAY);
				HAL_GPIO_WritePin(DE_RE_GPIO_Port, DE_RE_Pin, GPIO_PIN_RESET);
				Rx1Count = 0;
			}
			else ResetBuffers();
		}
		if (isReceiving == 1)
		{
			CheckTimeOut();
		}
		if (Rx2Count > 5 && isReceiving == 1)
		{
			lcd16x2_2ndLine();
			lcd16x2_printf("Num of S: %u", Rx2Count);
			uint16_t RequestCRC;
			RequestCRC = (Rx2Buffer[Rx2Count - 1] << 8) | Rx2Buffer[Rx2Count - 2];
			uint16_t CRCCalc = ModbusCalcCRC(Rx2Buffer, Rx2Count - 2);
			if (RequestCRC == CRCCalc)
			{
				HAL_UART_Transmit(&huart1, Rx2Buffer, Rx2Count, HAL_MAX_DELAY);
				ResetBuffers();
			}
			/*HAL_UART_Transmit(&huart1, Rx2Buffer, Rx2Count, HAL_MAX_DELAY);
			ResetBuffers();*/
			else ResetBuffers();
		}
		if (Rx2Count > 0 && isReceiving == 0)
		{
			Rx2Count = 0;
			memset(Rx2Buffer, 0, RX_BUFFER_SIZE);
		}
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
  huart2.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DE_RE_GPIO_Port, DE_RE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RS_Pin|E_Pin|D4_Pin|D5_Pin
                          |D6_Pin|D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DE_RE_Pin */
  GPIO_InitStruct.Pin = DE_RE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DE_RE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RS_Pin E_Pin D4_Pin D5_Pin
                           D6_Pin D7_Pin */
  GPIO_InitStruct.Pin = RS_Pin|E_Pin|D4_Pin|D5_Pin
                          |D6_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
