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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MSG_MAX_LEN 16

#define SCHERM_UART huart6
#define DEBUG_UART huart2
#define SLAVE_UART huart3

#define MSG_TIM htim2
#define LED_TIM htim5

#define ACK_TIMEOUT 500

#define LED_BLINK 3
#define LED_TOGGLE 2
#define LED_ON 1
#define LED_OFF 0

#define BLUE_LED ((uint16_t)0x8000)
#define RED_LED ((uint16_t)0x4000)
#define ORANGE_LED ((uint16_t)0x2000)
#define GREEN_LED ((uint16_t)0x1000)

enum state_t {
	STATE_STR,
	STATE_ADR,
	STATE_CHS,
	STATE_CTR,
	STATE_ERR,
	STATE_END,
};

#define debug

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
char CheckTimeout(const char * valid_ans, int timeout);
char SendMessage(const char * msg);
char SendCommand(const char * cmd, const char * ans, int timeout);
void SetLed(uint16_t led_pin, uint8_t state);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	uint8_t rx_buff[1];
	uint8_t tx_debug[12];	// Used only for debugging
	uint8_t rec_buff[MSG_MAX_LEN];
	uint8_t msg_i = 0;				// Index for rec_buff
	uint8_t msg_flag = 0;
	uint8_t msg_enable = 0;
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
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_USB_HOST_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&SLAVE_UART, rx_buff, 1);
  HAL_TIM_Base_Start_IT(&LED_TIM);

  uint8_t no_slaves = 0;
  enum state_t state = STATE_STR;
  uint16_t mode = 360;
  uint8_t score = 0;
  uint8_t chosen_button = 0;
  char buf[12];

  // Srand  needs a seed that differs each startup, no ideas for this have yet been implemented
  // Not as important, as the MCU won't be reset every time a person plays.
  // When the device loses and regains power however, the button sequence will always be the same
  srand(1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    switch(state)
    {
    case STATE_STR:
    	SetLed(GREEN_LED, LED_OFF);
    	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
    	{
    		SetLed(GREEN_LED, LED_ON);
    		state = STATE_ADR;
    	}
    	break;

    case STATE_ADR:
    	//SetLed(ORANGE_LED, LED_OFF);
    	if(SendCommand("0 ADR 1\n","SLAVE", ACK_TIMEOUT*2))
    	{
    		no_slaves = atoi((char*)&rec_buff[6]);
    		state = STATE_CHS;
    	}
    	else
    	{
    		SetLed(ORANGE_LED, LED_ON);
    		state = STATE_STR;
    	}
    	break;

//    	if(SendMessage("0 ADR 1\n"))
//    	{
//    		if(CheckTimeout("SLAVE", ACK_TIMEOUT*2))
//    		{
//    			no_slaves = atoi((char*)&rec_buff[6]);
//    			state = STATE_CHS;
//    			break;
//    		}
//    	}
//		SetLed(ORANGE_LED, LED_ON);
//		state = STATE_STR;

    	break;

    case STATE_CHS:
    	//TODO: print score
    	SetLed(RED_LED, LED_OFF);
    	if(no_slaves)
    		chosen_button = (rand() % (mode == 360 ? no_slaves : no_slaves >> 1))+1;

    	state = STATE_CTR;
    	break;

    case STATE_CTR:

    	sprintf(buf, "0 ON %d\n", chosen_button);
    	if(SendCommand(buf, "PRESSED", 500/*5000-(score*100)*/))
    	{
    		if(atoi((char*)&rec_buff[8]) == chosen_button)
    		{
				state = STATE_CHS;
				score++;
    		}
    		else
    			state = STATE_END;
    		SendMessage("0 OFF\n");
    	}
    	else
    		state = STATE_ERR;



//    	if(SendMessage((char*)buf))
//    	{
//    		if(CheckTimeout("PRESSED", 5000-(score*100)))
//    		{
//    			if(atoi((char*)&rec_buff[8]) == chosen_button)
//    			{
//    				state = STATE_CHS;
//    				score++;
//    			}
//    			else
//    				state = STATE_END;
//    		}
//    		else
//    			state = STATE_ERR;
//
//    		SendMessage("0 OFF");
//    	}
//    	else
//    	{
//    		SetLed(RED_LED, LED_ON);
//    		state = STATE_ERR;
//    	}
    	break;

    case STATE_ERR:

    	sprintf(buf, "%d ASK\n", chosen_button);
    	if(SendCommand(buf, "ANS", ACK_TIMEOUT*2))
    	{
			if(atoi((char*)&rec_buff[4]) == chosen_button)
				SetLed(RED_LED, LED_OFF);
    	}
    	else
    		SetLed(RED_LED, LED_ON);
    	state = STATE_END;
//    	if(SendMessage((char*)buf))
//    	{
//    		if(CheckTimeout("ANS", ACK_TIMEOUT))
//    		{
//    			if(atoi((char*)&rec_buff[4]) != chosen_button)
//    				SetLed(RED_LED, LED_ON);
//    		}
//    	}
//    	state = STATE_END;
    	break;

    case STATE_END:
    	//TODO: Print score
    	HAL_Delay(500); // Currently delay of 500ms, real delay should be 10 seconds
    	score = 0;
    	state = STATE_STR;
    	break;

    default:
    	state = STATE_STR;
    }
  }
  __NOP(); // Should never reach here

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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 42000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 200;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 41999;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 200;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// Master has defined answers its looking for
	// Only listens to the message if
	if(!msg_enable)
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
			memset(rec_buff, 0, sizeof(rec_buff));

    	SetLed(ORANGE_LED, LED_BLINK);

		rec_buff[msg_i] = rx_buff[0];
		if(rx_buff[0] == '\n')
		{
			if(huart == &SLAVE_UART && strcmp("ACK\n", (char*)rec_buff) != 0)
				HAL_UART_Transmit(&SLAVE_UART, (uint8_t*)"ACK\n", 4, 100);
			msg_i = 0;
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

char CheckTimeout(const char * valid_ans, int timeout)
{
	msg_enable = 1;		// Allows communication to the master
	int tim_cnt = 0;	// Keeps a count of how many times the timer updated, this happens every 100ms

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

	// Disables any other incoming messages
	msg_enable = 0;

	// Reads the incoming message (if received), and compares it to the expected answer
	if(msg_flag)
	{
		msg_flag = 0;
#ifdef debug
		HAL_UART_Transmit(&DEBUG_UART, (uint8_t*) "Received: ", 10, 100);
		HAL_UART_Transmit(&DEBUG_UART, (uint8_t*) rec_buff, strlen((char*)rec_buff), 100);
#endif
		if(strstr((char*)rec_buff, valid_ans) != NULL)
			return 1;
#ifdef debug
		else
			HAL_UART_Transmit(&DEBUG_UART, (uint8_t*) "Wrong Answer\n", 13, 100);
#endif
	}
#ifdef debug
	else
		HAL_UART_Transmit(&DEBUG_UART, (uint8_t*) "Timeout\n", 8, 100);
#endif

	// if the earlier return statement didn't fire, return 0, indicating either a timeout or a wrong answer
	return 0;		// implicit else

}

char SendMessage(const char * msg)
{
#ifdef debug
	HAL_UART_Transmit(&DEBUG_UART, (uint8_t*) "Sent: ", 6, 10);
	HAL_UART_Transmit(&DEBUG_UART, (uint8_t*) msg, strlen(msg), 100);
#endif

	// Every message sent requires "ACK\n" as an answer.
	// Master only sends messages to slaves this way, as the screen doesn't require acknowledgement
	SetLed(ORANGE_LED, LED_BLINK);
	strcpy((char*)tx_debug, (char*)msg);
	HAL_UART_Transmit(&SLAVE_UART, (uint8_t*) msg, strlen(msg), 100);
	return (CheckTimeout("ACK\n", ACK_TIMEOUT));
}

char SendCommand(const char * cmd, const char * ans, int timeout)
{
	if(SendMessage(cmd))
	{
		if(ans != NULL)
		{
			if(CheckTimeout(ans, timeout))
			{
				return 1;
			}
		}
		else
			return 1;
	}
	return 0;
}

void SetLed(uint16_t led_pin, uint8_t state)
{
	if(state == LED_TOGGLE)
		HAL_GPIO_TogglePin(GPIOD, led_pin);

	if(state == LED_ON)
    	HAL_GPIO_WritePin(GPIOD, led_pin, GPIO_PIN_SET);

	if(state == LED_OFF)
    	HAL_GPIO_WritePin(GPIOD, led_pin, GPIO_PIN_RESET);

	if(led_pin == ORANGE_LED && state == LED_BLINK)
	{
		HAL_GPIO_WritePin(GPIOD, led_pin, GPIO_PIN_SET);
		__HAL_RCC_TIM5_CLK_ENABLE();
		HAL_TIM_Base_Start_IT(&LED_TIM);
		__HAL_TIM_SET_COUNTER(&LED_TIM, 0);
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
