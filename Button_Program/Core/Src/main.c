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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MSG_MAX_LEN 12
#define MASTER_UART huart2
#define SLAVE_UART huart1
#define MSG_TIM htim1
#define ACK_TIMEOUT 500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
	uint8_t buf[] = "HOI \r\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx_buff[1];
uint8_t tx_debug[12];
uint8_t rec_buff[MSG_MAX_LEN];
uint8_t msg_i = 0;				// Index for rec_buff
uint8_t main_flag = 0;
uint8_t msg_flag = 0;
uint8_t msg_enable = 0;
uint8_t slave_adr = 0;
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&MASTER_UART, rx_buff, 1);
  HAL_UART_Receive_IT(&SLAVE_UART, rx_buff, 1);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if(main_flag)
	{

		main_flag = 0;
		msg_flag = 0;
		char reply[MSG_MAX_LEN];
		char cmd[4];
		char sec_adr;
		char adr = DisectCommand(cmd, &sec_adr);
		if(adr == 0)
		{
			if(strcmp(cmd, "ADR") == 0)
			{
				slave_adr = sec_adr;
				sprintf(reply, "0 ADR %d\n", slave_adr+1);
				if(SendMessage(&SLAVE_UART, reply) == 0)
				{
					sprintf(reply, "SLAVE %d\n", slave_adr);
					SendMessage(&MASTER_UART, reply);

					__NOP();
				}
				__NOP();
			}
			else if(strcmp(cmd, "ON") == 0)
			{
				// Sends message over to the next slave
				sprintf(reply, "0 ON %d\n", sec_adr);
				HAL_UART_Transmit(&SLAVE_UART, (uint8_t*) reply, strlen(reply), 100);

				// LED is low-active with pin C13
				if(sec_adr == slave_adr)
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

				// Checks if the button is being pressed
				char pressed = 0;
				while(pressed == GPIO_PIN_RESET && strcmp((char*)rec_buff, "0 OFF\n") != 0)
				{
					pressed = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
				}

				// Waits until the button is released
				while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1));

				// Forwards the message to neighbour.
				// Would be neater if this was done during the callback
				if(strcmp(rec_buff, "0 OFF\n") == 0)
				{
					SendMessage(&SLAVE_UART, "0 OFF\n");
				}

				// Only check if the button has been pressed if no message has been received yet.
				// Uses Transmit instead of SendMessage because master communication doesn't need ACK
				else if(pressed)
				{
					sprintf(reply, "PRESSED %d\n", slave_adr);
					HAL_UART_Transmit(&MASTER_UART, (uint8_t*) reply, strlen(reply), 100);
				}

				// Turns the LED off (low-active)
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			}

			// If the button was pressed, process wont be in the forwarding mode
			// Would be neater if this was done in callback function
			else if(strcmp(cmd, "OFF") == 0)
			{
				SendMessage(&SLAVE_UART, "0 OFF\n");
			}
		}
		else
		{
			if(strcmp((char*)cmd, "ASK") == 0)
			{
				sprintf(reply, "ANS %d\n", slave_adr);
				HAL_Delay(50); 					// The delay is because master doesn't always catch the message in time.
				SendMessage(&MASTER_UART, reply);
			}
			else
			{


			}
			__NOP();
		}


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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 32000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// Currently there exists a bug where the master sends out a null character after initializing.
	// This is used to capture and throw away said null character, while the bug still exists.
	if(rx_buff[0] == 0)
	{
		HAL_UART_Receive_IT(huart, rx_buff, 1);
		return;
	}

	// Used when message is out of sync with buffer
	static uint8_t OOS_check = 0;
	if(OOS_check)
	{
		if(rx_buff[0] == '\n')
			OOS_check = 0;
	}

	// Places new byte into global array
	else if(msg_i < MSG_MAX_LEN)
	{
		// Once a new message has started, delete the old message
		if(msg_i == 0)
		{
			msg_flag = 0;
			memset(rec_buff, 0, sizeof(rec_buff));
		}
		// Copying of new byte
		rec_buff[msg_i] = rx_buff[0];

		// Check if message is complete
		if(rx_buff[0] == '\n')
		{
			msg_i = 0;
			if((strcmp("ACK\n", (char*)rec_buff) != 0) && huart == &MASTER_UART)
			{

				HAL_UART_Transmit(huart, (uint8_t*)"ACK\n", 4, 100);
			}
			else if((strcmp("ACK\n", (char*)rec_buff) != 0) && huart == &SLAVE_UART)
			{
				HAL_UART_Transmit(&MASTER_UART, (uint8_t*)rec_buff, strlen((char*)rec_buff), 100);
				__NOP();
			}
			// Compares the message address the the address of the slave
			int msg_adr = atoi((char *)rec_buff);
			if(huart == &MASTER_UART)
			{
				if(msg_adr == 0 || msg_adr == slave_adr)
				{
					msg_flag = 1;
					main_flag = 1;
				}
				else
				{
					HAL_UART_Transmit(&SLAVE_UART, (uint8_t*)rec_buff, strlen((char*)rec_buff), 100);
					__NOP();
				}
			}
			if(strcmp("ACK\n", (char*)rec_buff) == 0)
				msg_flag = 1;
		}
		else
		{
			msg_i++;
		}
	}

	// Exception clause for handling messages larger than the dedicated buffer
	// Message will be discarded and the buffer won't be read
	else
	{
		msg_i = 0;
		if(rx_buff[0] != '\n')
			OOS_check = 1;
	}

	// Reads the next byte
	HAL_UART_Receive_IT(huart, rx_buff, 1);
}

char DisectCommand(char * cmd, char * sec_adr)
{
	// Reads the global message buffer and splits string up to three parts
	// These are the address, the command and the optional second address for advanced commands
	char * cmd_tok;

	char adr = atoi(strtok((char*)rec_buff, " "));
	cmd_tok = strtok(NULL, " ");
	*sec_adr = atoi(strtok(NULL, " "));

	sprintf(cmd, "%s", cmd_tok);
	cmd[3] = '\0';
	return adr;
}

char CheckTimeout(const char * valid_ans, int timeout)
{
	int tim_cnt = 0; // Allows communication to the master

	// Starts the timer for the timeout function.
	// Clears flag as a precaution
	HAL_TIM_Base_Start(&MSG_TIM);
	__HAL_TIM_CLEAR_FLAG(&MSG_TIM, TIM_FLAG_UPDATE);

	// When a message has been received "msg_flag" will be 1
	// This loop will count for "timeout" milliseconds
	while(msg_flag == 0 && tim_cnt < (int)((float)timeout/100))
	{
		if(__HAL_TIM_GET_FLAG(&MSG_TIM, TIM_FLAG_UPDATE))
		{
			tim_cnt++;
			__HAL_TIM_CLEAR_FLAG(&MSG_TIM, TIM_FLAG_UPDATE);
		}
	}

	// Stops the timer and resets the counter
	HAL_TIM_Base_Stop(&MSG_TIM);
	__HAL_TIM_SET_COUNTER(&MSG_TIM, 0);

	// Reads the incoming message (if received), and compares it to the expected answer
	if(msg_flag)
	{
		msg_flag = 0;
		if(strstr(valid_ans, (char*)rec_buff) != NULL)
			return 1;
	}

	// if the earlier return statement didn't fire, return 0, indicating either a timeout or a wrong answer
	return 0;		// implicit else

}

char SendMessage(UART_HandleTypeDef *huart, const char * msg)
{
	strcpy((char*)tx_debug, msg);
	if(HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100) != HAL_OK)
		__NOP();
	return (CheckTimeout("ACK\n", ACK_TIMEOUT));
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
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
