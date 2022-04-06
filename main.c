/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
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
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

CAN_RxHeaderTypeDef RxHeader;
uint32_t TxMailbox = 0;
uint8_t TxDataUART[16] = {0,};
uint8_t TxData[8] = {0,};
CAN_TxHeaderTypeDef TxHeader;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
void CAN_Transmit(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//сама функция передачи:

void CAN_Transmit(void)
{
	//TxHeader.StdId = 0x0378;
	TxHeader.ExtId = 0;
	TxHeader.RTR = CAN_RTR_DATA; //CAN_RTR_REMOTE
	TxHeader.IDE = CAN_ID_STD;   // CAN_ID_EXT
	//TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = 0;
	uint8_t TxData_Return[64] = "TTTT";// {0,}
	 if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	        {

		 HAL_UART_Transmit(&huart2, (uint8_t*)"ER SEND\n", 8, 100);
	        }
	 else {
		 sprintf(TxData_Return, "\nSEND %x ", TxHeader.StdId);
		 	HAL_UART_Transmit(&huart2, (uint8_t*)TxData_Return, strlen(TxData_Return), 100);
		 	for (int i = 0; i<TxHeader.DLC; i++ ) {
		 		sprintf(TxData_Return, "%x ", TxData[i]);
		 		HAL_UART_Transmit(&huart2, (uint8_t*)TxData_Return, strlen(TxData_Return), 100);

		 	}
		 	sprintf(TxData_Return, " \n");
		 	HAL_UART_Transmit(&huart2, (uint8_t*)TxData_Return, strlen(TxData_Return), 100);
	 }
}

int16_t  AsciiToDig(uint8_t c) {
	if ((c>='0')&&(c<='9')) return (c-'0');
	if ((c>='a')&&(c<='f')) return (c-'a'+10);
	if ((c>='A')&&(c<='F')) return (c-'A'+10);
	return -1;
}


uint8_t  parser_UART_CAN(uint8_t in_data)
{
	static uint8_t pos;
	int16_t ii = AsciiToDig (in_data);
	//int16_t byte_counter = 0;
	switch (pos) {
	//start input message
	case 0 :
		//clear CAN message
		TxHeader.StdId = 0;
		TxHeader.DLC = 0;
		for (uint8_t i=0; i<8; i++) TxData[i] = 0;
		//start of input string
		if (in_data == 'i') {
			pos = 1;
			//TxDataUART[0] = 'i';
			strcpy (TxDataUART, "i");
			break;
		}
		strcpy (TxDataUART, "not i\n");
		break;
	//input message id
	case 1 :
	case 2 :
	case 3 :
		if (ii<0) {
			pos = 0;
			strcpy (TxDataUART, "not digit\n");
			break;
		}
		TxDataUART[0]=in_data;
		if (pos==3) {TxDataUART[2]=0;TxDataUART[1]=' ';}
		else TxDataUART[1]=0;
		TxHeader.StdId = TxHeader.StdId*16 + ii;
		pos++;
		break;
	//input message data
	case 4 :
	case 5 :
	case 6 :
	case 7 :
	case 8 :
	case 9 :
	case 10 :
	case 11 :
	case 12 :
	case 13 :
	case 14 :
	case 15 :
	case 16 :
	case 17 :
	case 18 :
	case 19 :
		if (in_data == '\r') {
			pos = 0;
			TxDataUART[0] = 0;
			CAN_Transmit();
			break;
		}
		if (ii<0) {
			pos = 0;
			strcpy (TxDataUART, "not digit\n");
			break;
		}
		TxDataUART[0]=in_data;
		if (pos&1==1) {TxDataUART[2]=0;TxDataUART[1]=' ';}

		else {TxDataUART[1]=0;TxHeader.DLC++;}
		TxData[TxHeader.DLC-1] = TxData[TxHeader.DLC-1]*16 + ii;
		pos++;
		break;
	default:
		if (in_data == '\r') {
			pos = 0;
			TxDataUART[0] = 0;
			CAN_Transmit();
			break;
		}

		pos = 0;
		strcpy (TxDataUART, "not CR\n");
		break;


	}

	return in_data;
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
  MX_USART2_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //char * str = {};
	  //HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	  //HAL_Delay(200);
	  //HAL_UART_Receive(huart, pData, Size, Timeout)
	  if (( USART2->SR)& 0x20) {
		  unsigned int data0;
		  data0 = (USART2->DR)&0xff;
		  //USART2->DR = data0;
		  parser_UART_CAN(data0);
		  HAL_UART_Transmit(&huart2, (uint8_t*)TxDataUART, strlen(TxDataUART), 100);
	  }


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
  hcan.Init.Prescaler = 32;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
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

  /* USER CODE END CAN_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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

