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

#include <string.h>
#include <ctype.h>

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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint8_t count_l = 0; //счетчик левого датчика оборотов
uint8_t count_r = 0; //счетчик правого датчика оборотов

uint8_t buf[5]; //buf[0] - направление вращения, buf[1] - кол-во шагов
uint8_t command[4]; //команды для выполнения, аналогично buf
uint8_t delay;
uint8_t dataReceived = 0; // признак данное получено

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  HAL_UART_Receive_IT (&huart1, buf, 1);

	  if (buf[0] == ';')
	  {
		  HAL_GPIO_TogglePin(GPIOC, LED_Pin);

		  HAL_UART_Receive_IT (&huart1, buf, 2);

		  while( HAL_UART_GetState (&huart1) == HAL_UART_STATE_BUSY_RX );

		  command[0] = buf[0];
		  command[1] = buf[1];

		  dataReceived = 1;

	  }

	  if (dataReceived)
	  {
		  delay = (uint8_t)(command[1] - '0');

		  //HAL_UART_Transmit_IT (&huart1, buf, 2);

		  switch (command[0])
		  {
			  case '1': //forward

				  count_l = 0;
				  count_r = 0;
				  HAL_GPIO_WritePin(GPIOA, Motor_L_Pin, 1);

				  HAL_GPIO_WritePin(GPIOA, Motor_L_1_Pin, 1);
				  HAL_GPIO_WritePin(GPIOA, Motor_L_2_Pin, 0);

				  HAL_GPIO_WritePin(GPIOA, Motor_R_Pin, 1);

				  HAL_GPIO_WritePin(GPIOA, Motor_R_1_Pin, 1);
				  HAL_GPIO_WritePin(GPIOA, Motor_R_2_Pin, 0);


				  while (count_l < delay) {}
				  HAL_GPIO_WritePin(GPIOA, Motor_L_Pin, 0);
				  HAL_GPIO_WritePin(GPIOA, Motor_R_Pin, 0);

				  dataReceived = 0;
				  break;

			  case '2': //back
				  count_l = 0;
				  count_r = 0;
				  HAL_GPIO_WritePin(GPIOA, Motor_L_Pin, 1);

				  HAL_GPIO_WritePin(GPIOA, Motor_L_1_Pin, 0);
				  HAL_GPIO_WritePin(GPIOA, Motor_L_2_Pin, 1);

				  HAL_GPIO_WritePin(GPIOA, Motor_R_Pin, 1);

				  HAL_GPIO_WritePin(GPIOA, Motor_R_1_Pin, 0);
				  HAL_GPIO_WritePin(GPIOA, Motor_R_2_Pin, 1);


				  while (count_r < delay) {}
				  HAL_GPIO_WritePin(GPIOA, Motor_L_Pin, 0);
				  HAL_GPIO_WritePin(GPIOA, Motor_R_Pin, 0);

				  dataReceived = 0;
				  break;

			  case '3': //left

				  count_l = 0;
				  count_r = 0;
				  HAL_GPIO_WritePin(GPIOA, Motor_L_Pin, 1);

				  HAL_GPIO_WritePin(GPIOA, Motor_L_1_Pin, 0);
				  HAL_GPIO_WritePin(GPIOA, Motor_L_2_Pin, 0);

				  HAL_GPIO_WritePin(GPIOA, Motor_R_Pin, 1);

				  HAL_GPIO_WritePin(GPIOA, Motor_R_1_Pin, 1);
				  HAL_GPIO_WritePin(GPIOA, Motor_R_2_Pin, 0);


				  while (count_r < delay) {}
				  HAL_GPIO_WritePin(GPIOA, Motor_L_Pin, 0);
				  HAL_GPIO_WritePin(GPIOA, Motor_R_Pin, 0);

				  dataReceived = 0;
				  break;

			  case '4': //right

				  count_l = 0;
				  count_r = 0;
				  HAL_GPIO_WritePin(GPIOA, Motor_L_Pin, 1);

				  HAL_GPIO_WritePin(GPIOA, Motor_L_1_Pin, 1);
				  HAL_GPIO_WritePin(GPIOA, Motor_L_2_Pin, 0);

				  HAL_GPIO_WritePin(GPIOA, Motor_R_Pin, 1);

				  HAL_GPIO_WritePin(GPIOA, Motor_R_1_Pin, 0);
				  HAL_GPIO_WritePin(GPIOA, Motor_R_2_Pin, 0);


				  while (count_l < delay) {}
				  HAL_GPIO_WritePin(GPIOA, Motor_L_Pin, 0);
				  HAL_GPIO_WritePin(GPIOA, Motor_R_Pin, 0);

				  dataReceived = 0;
				  break;
		  }
		  //huart1.Init.Mode = UART_MODE_TX_RX;
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Motor_R_Pin|Motor_L_Pin|Motor_L_2_Pin|Motor_L_1_Pin
                          |Motor_R_1_Pin|Motor_R_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor_R_Pin Motor_L_Pin Motor_L_2_Pin Motor_L_1_Pin
                           Motor_R_1_Pin Motor_R_2_Pin */
  GPIO_InitStruct.Pin = Motor_R_Pin|Motor_L_Pin|Motor_L_2_Pin|Motor_L_1_Pin
                          |Motor_R_1_Pin|Motor_R_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : R_Sensor_Pin L_Sensor_Pin */
  GPIO_InitStruct.Pin = R_Sensor_Pin|L_Sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

  if(huart == &huart1) {

    dataReceived = 1;

    command[0] = buf[0];
    command[1] = buf[1];

    //str[3] = 0;//end \0 in string

    //HAL_UART_Transmit_IT (&huart1, str, 1);

    if (!strcmp(str, "1"))
    {
    	HAL_GPIO_WritePin(GPIOC, LED_Pin, 1);
    }
    else if (!strcmp(str, "0"))
	{
		HAL_GPIO_WritePin(GPIOC, LED_Pin, 0);
	}*/

//    HAL_UART_Receive_IT (&huart1, buf, 1);

//  }
//}



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
