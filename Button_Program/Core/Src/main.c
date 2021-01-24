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
/**
  * @brief 	Struct used for separating screen and slave module.
  */
typedef struct {
	uint8_t msg_i;						/*!< Index for received messages 				*/

	uint8_t msg_flag;					/*!< Message flag to indicate a new message 	*/

	uint8_t rec_buff[MSG_MAX_LEN];		/*!< Buffer for the received message			*/

	UART_HandleTypeDef * used_huart;	/*!< Pointer to the UART struct the module uses */
}t_module;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/** @defgroup Used_Peripherials Used Peripherals
  * @{
  */
#define MASTER_UART 				huart2			/*!< UART  peripheral connected to the master 				*/
#define SLAVE_UART 					huart1			/*!< UART  peripheral connected to the slaves 				*/
#define SPEAKER_TIM 				htim2			/*!< Timer peripheral for generating PWM to the speaker		*/
#define MSG_TIM 					htim1			/*!< Timer peripheral used for message timeout checking 	*/
/**
  * @}
  */

/** @defgroup Message_Defines Message Related Defines
  * @{
  */
#define MASTER_I 					0				/*!< The index for the master module 									*/
#define SLAVE_I 					1				/*!< The index for the slave  module 									*/
#define ACK_TIMEOUT 				500				/*!< How long the slave waits for an acknowledge from another slave		*/
#define CMD_MAX_LEN 				3				/*!< How many chars in the message buffer are reserved for the command  */
/**
  * @}
  */

/** @defgroup Button_GPIO Button GPIO Connections and Pins
  * @{
  */
#define BT_GPIO_1 					GPIOA			/*!< GPIO for button 1 */
#define BT_GPIO_2 					GPIOB			/*!< GPIO for button 2 */
#define BT_GPIO_3 					GPIOB			/*!< GPIO for button 3 */

#define BT_PIN_1 					GPIO_PIN_7		/*!< Pin for button 1 */
#define BT_PIN_2 					GPIO_PIN_0		/*!< Pin for button 2 */
#define BT_PIN_3 					GPIO_PIN_1		/*!< Pin for button 3 */
/**
  * @}
  */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/** @brief rx_buff contains the incoming char */
uint8_t rx_buff[1];
/** @brief flag that enables main */
uint8_t main_flag = 0;
/** @brief the address of this slave */
uint8_t slave_adr = 0;
/** @brief m_buff are the two buffers for screen/slave message handling*/
t_module m_buff[2];

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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  m_buff[MASTER_I].used_huart = &MASTER_UART;
  m_buff[SLAVE_I].used_huart  = &SLAVE_UART;

  HAL_UART_Receive_IT(&MASTER_UART, rx_buff, 1);
  HAL_UART_Receive_IT(&SLAVE_UART , rx_buff, 1);
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
		m_buff[MASTER_I].msg_flag = 0;
		/** <h2><center>Local Variables</h2></center> */
		/** <b>char reply</b>
				 Container for the reply message to the master.*/
		char reply[MSG_MAX_LEN];

		/** <b>char m_mess</b>
				 Container for messages to the next slave.*/
		char m_mess[MSG_MAX_LEN];

		/** <b>char cmd</b>
				 Container for the dissected command.*/
		char cmd[CMD_MAX_LEN+1];

		/** <b>char sec_adr</b>
				 Used to store the secondary address stored in some messages.*/
		char sec_adr;


		// Dissects the message that arrived from the master side.
		char adr = DissectCommand(cmd, &sec_adr);
		if(adr == 0)
		{
			/** <h2><center>Command Functions and Replies</h2></center> */
			/** <h3>Addressing Command</h3>
			 *  Format: 			<tt>0 ADR x</tt>\n
			 *  Forward to Slave: 	<tt>0 ADR x+1</tt>\n
			 *  Reply to Master:	<tt>SLAVE x</tt>, But only if Reply to Slave didn't receive ACK.\n
			 *  					Will copy the address specified in @c x into its own address buffer @c slave_adr.\n
			 *  					Will then send command with @c x+1 to the next slave.
			 *  					If an @c ACK isn't received in time, will reply to the master instead.\n
			 */
			if(strcmp(cmd, "ADR") == 0)
			{
				slave_adr = sec_adr;
				sprintf(m_mess, "0 ADR %d\n", slave_adr+1);
				if(SendMessage(SLAVE_I, m_mess) == 0)
				{
					sprintf(reply, "SLAVE %d\n", slave_adr);
					SendMessage(MASTER_I, reply);
				}
			}
			/** <h3>Turn On Command</h3>
			 *  Format:				<tt>0 ON n</tt>\n
			 *  Forward to Slave:	<tt>0 ON n</tt>\n
			 *  Reply to Master:	<tt>PRESSED x</tt> But only when the button is pressed.\n
			 *  					Will activate the buttons and if @c slave_adr is the same as @c n will activate buzzer.
			 *  					Always forwards the message to the next slave. Doesn't require ACK checking so uses HAL.\n
			 *  					Will deactivate if a message is received from the master, or send <tt>PRESSED x</tt>.\n
			 *
						 */
			else if(strcmp(cmd, "ON") == 0)
			{
				// Sends message over to the next slave
				sprintf(m_mess, "0 ON %d\n", sec_adr);
				HAL_UART_Transmit(&SLAVE_UART, (uint8_t* ) m_mess, strlen(m_mess), 100);

				// Activates the speaker PWM
				if(sec_adr == slave_adr)
					HAL_TIM_PWM_Start(&SPEAKER_TIM, TIM_CHANNEL_1);

				// Checks if the button is being pressed
				char pressed = 0;
				while(pressed == GPIO_PIN_RESET && main_flag == 0)
					pressed = CheckButton();

				// Waits until the button is released
				while(CheckButton());

				// Only check if the button has been pressed if no message has been received yet.
				// Uses Transmit instead of SendMessage because master communication doesn't need ACK
				if(pressed)
				{
					HAL_Delay(50);
					sprintf(reply, "PRESSED %d\n", slave_adr);
					HAL_UART_Transmit(&MASTER_UART, (uint8_t*) reply, strlen(reply), 100);
				}

				// Turn the speaker Off
				HAL_TIM_PWM_Stop(&SPEAKER_TIM, TIM_CHANNEL_1);
			}

			/** <h3>Turn Off Command</h3>
			 * 	Format:				<tt>0 OFF</tt>\n
			 * 	Forward to Slave:	<tt>0 OFF</tt>\n
			 * 	Reply to Master	:	None\n
			 * 						Will copy the message to the next slaves.
			 * 						Used as placeholder to turn off the slaves after the ON command has been sent.\n
			 *
			 */
			else if(strcmp(cmd, "OFF") == 0)
				SendMessage(SLAVE_I, "0 OFF\n");
		}
		else
		{
			/** <h3>Verify Connection Command</h3>
			 * 	Format:				<tt>x ASK</tt>\n
			 * 	Forward to Slave:	None\n
			 * 	Reply to Master :	<tt>ANS x</tt>\n
			 * 						Will reply to the master personally to verify the connection.\n
			 *
			 */
			if(strcmp((char*)cmd, "ASK") == 0)
			{
				sprintf(reply, "ANS %d\n", slave_adr);
				HAL_Delay(50); 					// The delay is because master doesn't always catch the message in time.
				SendMessage(MASTER_I, reply);
			}
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  /** Initializes the CPU, AHB and APB buses clocks
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 32000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
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
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
 * @fn void HAL_UART_RxCpltCallback(UART_HandleTypeDef*)
 * @brief Called when a character is received from a uart.
 * 		  This char is stored as an array of rx_buff, with an index of 1.
 * 		  Will put the char in the correct buffer and check if the message is complete.
 * 		  Will reactivate the uart after char is handled.
 * @pre	  UARTS are turned on and waiting for interrupt.
 * @post  Char is stored and UART called under interrupt again.
 * @param huart The huart that called the callback
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// Selects the correct module struct to copy the char into.
	// This is marked by "ix" which is index.
	uint8_t ix;
	if(huart == m_buff[MASTER_I].used_huart)
		ix = MASTER_I;

	else if(huart == m_buff[SLAVE_I].used_huart)
		ix = SLAVE_I;

	else
	{
		HAL_UART_Receive_IT(huart, rx_buff, 1);		// Always has to be called after finishing the callback
		return;
	}
	// Currently there exists a bug where the master sends out a null character after initializing.
	// This is used to capture and throw away said null character, while the bug still exists.
	if(rx_buff[0] == 0)
	{
		HAL_UART_Receive_IT(huart, rx_buff, 1);
		return;
	}

	// Used when message is out of sync with buffer size
	static uint8_t OOS_check = 0;
	if(OOS_check)
	{
		if(rx_buff[0] == '\n')
			OOS_check = 0;
	}

	// Places new byte into global array
	else if(m_buff[ix].msg_i < MSG_MAX_LEN)
	{
		// Once a new message has started, delete the old message
		if(m_buff[ix].msg_i == 0)
		{
			m_buff[ix].msg_flag = 0;
			memset(m_buff[ix].rec_buff, 0, sizeof(m_buff[ix].rec_buff));
		}

		// Copying of new byte
		m_buff[ix].rec_buff[m_buff[ix].msg_i] = rx_buff[0];

		// Check if message is complete
		if(rx_buff[0] == '\n')
		{
			// Resets the index of the message to the first value 0
			m_buff[ix].msg_i = 0;

			// Activates the message flag required by CheckTimeout if the message was an ack.
			// Doesn't activate the main flag however, as it is not a command that needs responding to.
			if(strcmp("ACK\n", (char *) m_buff[ix].rec_buff) == 0)
				m_buff[ix].msg_flag = 1;

			// If the master sent a message...
			else if(m_buff[ix].used_huart == &MASTER_UART)
			{
				// Returns an ACK
				HAL_UART_Transmit(huart, (uint8_t*)"ACK\n", 4, 100);

				// Gets the address out of the message
				int msg_adr = atoi((char *) m_buff[ix].rec_buff);

				// If the address of the message isn't 0 or equal to the slaves address, the message will be sent to the next slave.
				if(msg_adr != 0 && msg_adr != slave_adr)
					HAL_UART_Transmit(&SLAVE_UART, (uint8_t *) m_buff[ix].rec_buff, strlen( (char *) m_buff[ix].rec_buff), 100);

				else
				{
					m_buff[ix].msg_flag = 1;	// Used by CheckTimeout
					main_flag = 1;				// Used to answer commands in the main
				}
			}
			// If a slave sent the message, it will always go to the master side.
			else if(m_buff[ix].used_huart == &SLAVE_UART)
				HAL_UART_Transmit(&MASTER_UART, (uint8_t *) m_buff[ix].rec_buff, strlen( (char *) m_buff[ix].rec_buff), 100);
		}
		else
			m_buff[ix].msg_i++;		// Index for the message buffer
	}

	// Exception clause for handling messages larger than the dedicated buffer
	// Message will be discarded and the buffer won't be read
	else
	{
		m_buff[ix].msg_i = 0;
		if(rx_buff[0] != '\n')
			OOS_check = 1;
	}

	// Reads the next byte
	HAL_UART_Receive_IT(huart, rx_buff, 1);
}

/**
 * @fn char DissectCommand(char*, char*)
 * @brief 			DissectCommand takes the command as input (from the global buffer) and returns the address and command letters.
 * 					If the command contained a second address it will be written inside sec_adr.
 *
 * @pre 			A command targeted at this specific slave arrived from the master module side.
 * @post 			The message is dissected and its component parts stored in the relevant variables.
 * @param cmd		The address where the command letters will be stored once finished
 * @param sec_adr	The address where the second address will be stored if found.
 * @return			The address found at the start of the message.
 */
char DissectCommand(char * cmd, char * sec_adr)
{
	// Reads the global master message buffer and splits string up to three parts
	// These are the address, the command and the optional second address for advanced commands
	char * cmd_tok;

	// strtok splits the message for every time it encounters a used defined char.
	// In this case the user defined char is a space character, because all info in the message is split by spaces.
	char adr = atoi(strtok( (char *) m_buff[MASTER_I].rec_buff, " ") );	// Atoi returns the number at the start (address)
	cmd_tok = strtok(NULL, " ");										// Gets the letters of the command (cmd)
	*sec_adr = atoi(strtok(NULL, " ") );								// Gets the letters of the second address

	// To make sure string operations can be performed on the command, a 0 character is added to the end of it.
	// This does mean commands are now limited to 3 characters because this is now hard coded in here.
	sprintf(cmd, "%s", cmd_tok);
	cmd[CMD_MAX_LEN] = '\0';
	return adr;
}

/**
 * @fn char CheckTimeout(const char*, int, uint8_t)
 * @brief			Function will be called when a message is expected.
 * 					Used to verify the answer received from the specified module.
 * 					Also uses a timer to check for a timeout.
 * @pre 			Message has been sent that expects a reply
 * @post 			Answer has been received from the correct module and verified.
 * @param valid_ans The expected answer from the selected module.
 * @param timeout	The time the selected module has to answer. Uses increments of 100ms
 * @param ix		The index for the m_buff array, used to select the module.
 * @return			1 if correct reply received in time, 0 if not.
 */
char CheckTimeout(const char * valid_ans, int timeout, uint8_t ix)
{
	// Starts the timer for the timeout function.
	// Clears flag as a precaution
	HAL_TIM_Base_Start(&MSG_TIM);
	__HAL_TIM_CLEAR_FLAG(&MSG_TIM, TIM_FLAG_UPDATE);

	// Keeps a count of how many times the timer updated
	// This happens every 100ms and is defined by MSG_TIM settings
	int tim_cnt = 0;

	// When a message has been received "msg_flag" will be 1
	// This loop will count for "timeout" milliseconds
	while(m_buff[ix].msg_flag == 0 && tim_cnt < (int) ( (float) timeout/100) )
	{
		if(__HAL_TIM_GET_FLAG(&MSG_TIM, TIM_FLAG_UPDATE) )
		{
			tim_cnt++;
			__HAL_TIM_CLEAR_FLAG(&MSG_TIM, TIM_FLAG_UPDATE);
		}
	}

	// Stops the timer and resets the counter
	HAL_TIM_Base_Stop(	  &MSG_TIM	 );
	__HAL_TIM_SET_COUNTER(&MSG_TIM, 0);

	// Reads the incoming message (if received), and compares it to the expected answer
	if(m_buff[ix].msg_flag)
	{
		m_buff[ix].msg_flag = 0;	// To prevent reading the same message multiple times.

		// Some commands expect an unknown number (i.e. how many slaves are active)
		// So this command only checks if the received message CONTAINS the correct answer.
		// To make sure it checks the message verbatim, add a \n to the end of the expected answer.
		if(strstr(valid_ans, (char *) m_buff[ix].rec_buff) != NULL)
			return 1;
	}

	// if the earlier return statement didn't fire, return 0, indicating either a timeout or a wrong answer
	return 0;

}

/**
 * @fn char SendMessage(uint8_t, const char*)
 * @brief 		Sends a message to the slaves and waits for an acknowledge.
 *
 * @pre 		none
 * @post 		Message has been sent
 * @param ix	The index indicating which module to send the message to. (Defined as MASTER_I or SLAVE_I)
 * @param msg	The message to send to the selected module.
 * @return		Returns 1 if ACK received, 0 if not.
 */
char SendMessage(uint8_t ix, const char * msg)
{
	// Every message sent to other slaves requires "ACK\n" as an answer.
	HAL_UART_Transmit(m_buff[ix].used_huart, (uint8_t *) msg, strlen(msg), 100);
	return (CheckTimeout("ACK\n", ACK_TIMEOUT, ix));
}

/**
 * @fn char CheckButton()
 * @brief 	Checks each pin connected to the button.
 * 			The defines for this are messy, as the pins are placed on GPIOA and GPIOB.
 * @pre 	The "ON" message was received.
 * @post 	The button is either pressed or not pressed.
 * @return	Non-zero if pressed. Zero if not.
 */
char CheckButton()
{
	char pressed = 0;
	pressed = HAL_GPIO_ReadPin(BT_GPIO_1, BT_PIN_1);
	pressed += HAL_GPIO_ReadPin(BT_GPIO_2, BT_PIN_2);
	pressed += HAL_GPIO_ReadPin(BT_GPIO_3, BT_PIN_3);
	return pressed;
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
