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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
  * @brief 	Struct used for separating screen and slave module.
  */
typedef struct {
	uint8_t msg_i;						/*!< Index for received messages 				*/

	uint8_t msg_en;						/*!< Enable flag to prevent reading while busy 	*/

	uint8_t msg_flag;					/*!< Message flag to indicate a new message 	*/

	uint8_t rec_buff[MSG_MAX_LEN];		/*!< Buffer for the received message			*/

	UART_HandleTypeDef * used_huart;	/*!< Pointer to the UART struct the module uses */
}t_module;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/** @defgroup Mode_Select Input/Output of CheckMsg() function
  * @{
  */
#define HALF 					"180\n"		/*!< String that indicates only half of the slaves must be used.	*/

#define HALF_NUM 				1			/*!< Value to indicate half of the buttons are used 			 	*/
#define FULL_NUM 				2			/*!< Value to indicate all  of the buttons are used					*/
/**
  * @}
  */

/** @defgroup Used_Peripherials Used Peripherals
  * @{
  */
#define SCHERM_UART 			huart2		/*!< UART  peripheral connected to the screen 				*/
#define SLAVE_UART 				huart1		/*!< UART  peripheral connected to the slaves 				*/
//#define DEBUG_UART 			huart2		/*!< UART  peripheral connected to debug PC	 				*/
#define MSG_TIM 				htim1		/*!< Timer peripheral used for message timeout checking 	*/
#define LED_TIM 				htim2		/*!< Timer peripheral used for blinking LEDS non-blocking 	*/
/**
  * @}
  */

/** @defgroup Timing_Definitions Delay/Timeout times in ms
  * @{
  */
#define ACK_TIMEOUT 			1000		/*!< How long the master waits for an acknowledge from a slave 			*/
#define SCORE_DELAY 			10000		/*!< How long the master waits for clearing the score					*/
#define START_TIME 				5000		/*!< Initial time the user has to press a button						*/
#define TIME_INCREMENT 			100			/*!< How much every point of score subtracts from the total time the	\
												 user has to press the button										*/
#define NEXT_BT_TIME			500			/*!< The amount of time the master waits between each button activation */
/**
  * @}
  */

/** @defgroup LED_Commands LED Command Definitions
  * @{
  */
#define LED_BLINK 				3			/*!< The command to blink an LED 	*/
#define LED_TOGGLE 				2			/*!< The command to toggle an LED 	*/
#define LED_ON 					0			/*!< The command to turn on an LED	*/
#define LED_OFF 				1			/*!< The command to turn off an LED */
/**
  * @}
  */

/** @defgroup Buffer_Indexes Buffer Indexes for m_buff
  * @{
  */
#define SCHERM_I 				0			/*!< The index for the screen module */
#define SLAVE_I 				1			/*!< The index for the slave module  */
/**
  * @}
  */

/** @defgroup LED_Pin_Definitions LED Pin Definitions
  * @{
  */
#define BLUE_LED 				((uint16_t)0x2000)	/*!< The blue LED pin */
/**
  * @}
  */

/**
 * @enum state_t
 * @brief The various states that the master module can be in.
 *
 */
enum state_t {
	STATE_STR,			/**< Start State				*/
	STATE_ADR,			/**< Addressing State 			*/
	STATE_CHS,			/**< Choosing State			  	*/
	STATE_CTR,			/**< Control and Verify State 	*/
	STATE_ERR,			/**< Error State 				*/
	STATE_END,			/**< Ending State				*/
};

#ifdef DEBUG_UART
#define debug			/*!< Debug enable */
#endif
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
char CheckTimeout(const char * valid_ans, int timeout);
char SendMessage(const char * msg);
char SendCommand(const char * cmd, const char * ans, int timeout);
void SetLed(uint16_t led_pin, uint8_t state);
char CheckMsg();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef debug
uint8_t tx_debug[12];	// Used only for debugging
#endif

/** @brief rx_buff contains the incoming char */
uint8_t rx_buff[1];
/** @brief blink_pin tells LED timer which pin to turn off */
uint16_t blink_pin;
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
#ifdef DEBUG_UART
#if DEBUG_UART != SCHERM_UART
  HAL_UART_Receive_IT(&DEBUG_UART, rx_buff, 1);
#endif
#endif
  HAL_UART_Receive_IT(&SLAVE_UART, 	rx_buff, 1);
  HAL_UART_Receive_IT(&SCHERM_UART, rx_buff, 1);
  HAL_TIM_Base_Start_IT(&LED_TIM);

  m_buff[SCHERM_I].used_huart = &SCHERM_UART;
  m_buff[SLAVE_I].used_huart  = &SLAVE_UART;

  uint8_t no_slaves  	= 0;
  enum state_t state	= STATE_STR;
  uint16_t mode 	 	= FULL_NUM;
  uint8_t score 	 	= 0;
  uint8_t chosen_button = 0;
  char buf[12];
  /**   <h2><center> Supported Commands </h2></center>
   	*   <h3>Addressing</h3>
   	*   Format: 	 <tt>0 ADR 1</tt>\n
   	*   Definitions: @c 0 sends to @b all slaves. @c ADR is the addressing command. @c 1 is the first address.\n\n
   	*   Used to give every slave a specific address down the chain.\n
   	*
   	*   <h3>Turning On Slaves</h3>
   	*   Format: 	 <tt>0 ON adr</tt>\n
   	*   Definitions: @c 0 sends to @b all slaves. @c ON is the turn on command. @c adr specifies which button is correct.\n\n
   	*   This turns on the buttons of all slaves. Only the @c adr slave will have their buzzer ring.\n
   	*
   	*   <h3>Turning Off Slaves</h3>
   	*   Format:		 <tt>0 OFF</tt>\n
   	*   Definitions: @c 0 sends to @b all slaves. @c OFF is the turn off command.\n\n
   	*   			 This command turns off all slaves, used @b AFTER the ON command.\n
   	*   			 In reality, the slaves don't listen to the specific message, any message will do to turn them off.\n
   	*   			 This is to prevent a missed "off" command to put the slave in blocking mode.\n
   	*   			 The alternative would have been to set a timer in the slave to quit.\n
   	*   			 This however decreases the amount of control the master could have over the slaves.\n
   	*   			 This compromise had to be made, so that the master is still in control.\n
   	*
   	*   <h3>Verify Connection</h3>
   	*   Format:		 <tt>adr ASK</tt>
   	*   Definitions: @c adr specifies which slave to send the command to. @c ASK is the command.\n
   	*   Explanation: This command asks a specific slave to reply with an answer.\n
   	*   			 This is used to verify the slave is still connected.\n
   	*   			 \n\n
    */

  //Needs a seed that differs each startup
  //Not as important, as the MCU won't be reset every time a person plays.
  //When the device loses and regains power however, the button sequence will always be the same
  srand(1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	/** <h2><center> State Machine </h2></center> */
    /* USER CODE BEGIN 3 */
	switch(state)
	  {
	  /**  <h3>Start State</h3>
	    *  @brief Reads incoming messages from the screen module.
	    *  		  Will switch to addressing state if message is received.
	    *  		  Mode will indicate if full range of buttons is to be used or only half.
	    */
	  case STATE_STR:
		m_buff[SCHERM_I].msg_en = 1;
		mode = CheckMsg();
		if(mode != 0)
		{
			state = STATE_ADR;
			m_buff[SCHERM_I].msg_en   = 0;
			m_buff[SCHERM_I].msg_flag = 0;
		}
		break;
	  /**  <h3>Addressing State</h3>
		*  @brief Will send a command to give an adress to each slave.
		*  		  If received "slave" back addressing successful.
		*  		  State will move to Choose if successful, Start if failed.
		*/
	  case STATE_ADR:
		if(SendCommand("0 ADR 1\n","SLAVE", ACK_TIMEOUT*2))
		{
			no_slaves = atoi( (char*) &m_buff[SLAVE_I].rec_buff[6]);
			state = STATE_CHS;
		}
		else
			state = STATE_STR;
		break;
	  /**  <h3>Choosing State</h3>
		*  @brief Will choose a random button based on mode.
		*  		  Mode was set by start state earlier.
		*  		  Has a check to make sure there won't be a div/0 error.
		*  		  Next state will be control or start depending on number of slaves.
		*/
	  case STATE_CHS:
		sprintf(buf, "SCORE =%d\r\n", score);
		HAL_UART_Transmit(&SCHERM_UART, (uint8_t *) buf, strlen( (char *) buf), 100);
		if(no_slaves)
		{
			// The +1 makes sure 0 is never a chosen button. Slaves start from address 1.
			// This can be made into a define, if there exists a desire to start from slave 2.
			chosen_button = ( rand() % (mode == FULL_NUM ? no_slaves : no_slaves >> 1))+1;
			state = STATE_CTR;
		}
		else
			state = STATE_STR;
		break;
	  /**  <h3>Control & Verify State</h3>
		*  @brief Controls the chosen button from Choose State.
		*  		  Sends out a message for all buttons to turn on.
		*  		  Specifies the address of chosen button to turn on its speakers.
		*		  Reads which button is pressed, or if a timeout occurred.
		*		  If the correct button is pressed, will choose a new button (state choose).
		*		  If the button was incorrect, will end the game (state end).
		*		  If no button was pressed in time, will verify connection (state error).
		*/
	  case STATE_CTR:
		sprintf(buf, "0 ON %d\n", chosen_button);
		if(SendCommand(buf, "PRESSED", START_TIME - ( score * TIME_INCREMENT) ) )
		{
			// The reply is PRESSED n, so to read the received number the 8th byte will be used.
			// P|R|E|S|S|E|D| |n
			// 0|1|2|3|4|5|6|7|8
			if(atoi( (char *) &m_buff[SLAVE_I].rec_buff[8]) == chosen_button)
			{
				state = STATE_CHS;
				score++;
			}
			else
				state = STATE_END;
		}
		else
			state = STATE_ERR;

		SendMessage("0 OFF\n");
		HAL_Delay(NEXT_BT_TIME);
		break;
	  /**  <h3>Error State</h3>
		*  @brief Error state will verify communication works with the chosen slave.
		*  		  If the slave does not answer to this check, an LED will indicate an error.
		*  		  No matter the result of the Comm test, the next state will always be END.
		*/
	  case STATE_ERR:

		sprintf(buf, "%d ASK\n", chosen_button);
		if(SendCommand(buf, "ANS", ACK_TIMEOUT*2))
		{
			// The answer is ANS n, so to read the received number the 4th byte will be used.
			// A|N|S| |n
			// 0|1|2|3|4
			if(atoi( (char *) &m_buff[SLAVE_I].rec_buff[4] ) == chosen_button)
				SetLed(BLUE_LED, LED_OFF);
		}
		else
			SetLed(BLUE_LED, LED_ON);
		state = STATE_END;
		break;
	  /**  <h3>End State</h3>
		*  @brief The final state of the program.
		*  		  Will display the score once and then reset it, along with the screen.
		*  		  Next state will be returning to start.
		*/
	  case STATE_END:
		sprintf(buf, "SCORE =%d\r\n", score);
		HAL_UART_Transmit(&SCHERM_UART, (uint8_t *) buf, strlen( (char *) buf), 100);
		HAL_Delay(SCORE_DELAY);

#ifdef debug
	sprintf(buf, "Score = %d\n", score);
	HAL_UART_Transmit(&DEBUG_UART, (uint8_t *)"Klaar met uitvoering.\r\n", 23, 100);
	HAL_UART_Transmit(&DEBUG_UART, (uint8_t *)buf, strlen((char*)buf), 100);
#endif

		HAL_UART_Transmit(&SCHERM_UART, (uint8_t *) "MENU\r\n", 6, 100);
		score = 0;
		state = STATE_STR;
		break;

	  default:
		state = STATE_STR;
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

  /*Configure GPIO pins : PA0 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_7;
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
	if(		huart == m_buff[SCHERM_I].used_huart)
		ix = SCHERM_I;

	else if(huart == m_buff[SLAVE_I].used_huart)
		ix = SLAVE_I;

	else
	{
		HAL_UART_Receive_IT(huart, rx_buff, 1);
		return;
	}

	// Master has defined answers its looking for
	// Only listens to the message if a command has been sent
	if(m_buff[ix].msg_en == 0)
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
	else if(m_buff[ix].msg_i < MSG_MAX_LEN)
	{
		// Once a new message has started, delete the old message
		if(m_buff[ix].msg_i == 0)
			memset(m_buff[ix].rec_buff, 0, sizeof(m_buff[ix].rec_buff));

		// Comm blink
    	SetLed(BLUE_LED, LED_BLINK);

    	// Coppies char into local buffer
    	m_buff[ix].rec_buff[m_buff[ix].msg_i] = rx_buff[0];

    	// Checks if message is finished
    	if(rx_buff[0] == '\n')
		{
    		// Will send an Acknowledge to the slave
    		// Not necessary, as the slave can't do anything if the connection with the master is broken.
			if(huart == &SLAVE_UART && strcmp("ACK\n", (char *) m_buff[ix].rec_buff) != 0)
				HAL_UART_Transmit(&SLAVE_UART, (uint8_t *) "ACK\n", 4, 100);
			m_buff[ix].msg_i 	= 0;
			m_buff[ix].msg_flag = 1;
		}
		else
		{
			m_buff[ix].msg_i++;
		}
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
 * @fn char CheckTimeout(const char*, int)
 * @brief	Function will be called when a message is expected.
 * 			Used to verify the answer received from the slave module.
 * 			Also uses a timer to check for a timeout.
 *
 * @pre		Message has been sent that requires a reply.
 * @post	Answer has been received and verified.
 * @param valid_ans The expected answer for the slave module.
 * @param timeout 	The time the slave has to give an answer. Uses increments of 100ms.
 * @return  Returns a non-zero character if correct message arrived in time.
 * 			Returns zero if the message was incorrect or it didn't arrive in time.
 */
char CheckTimeout(const char * valid_ans, int timeout)
{
	m_buff[SLAVE_I].msg_en = 1;		// Allows communication to the master

	// Keeps a count of how many times the timer updated
	// This happens every 100ms and is defined by MSG_TIM settings
	int tim_cnt = 0;

	// Starts the timer for the timeout function.
	// Clears flag as a precaution
	HAL_TIM_Base_Start(&MSG_TIM);
	__HAL_TIM_CLEAR_FLAG(&MSG_TIM, TIM_FLAG_UPDATE);

	// When a message has been received "msg_flag" will be 1
	// This loop will count for "timeout" milliseconds
	while(m_buff[SLAVE_I].msg_flag == 0 && tim_cnt < (int)((float)timeout/100))
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
	m_buff[SLAVE_I].msg_en = 0;

	// Reads the incoming message (if received), and compares it to the expected answer
	if(m_buff[SLAVE_I].msg_flag)
	{
		m_buff[SLAVE_I].msg_flag = 0;
#ifdef debug
		HAL_UART_Transmit(&DEBUG_UART, (uint8_t *) "Received: ", 10, 100);
		HAL_UART_Transmit(&DEBUG_UART, (uint8_t *) m_buff[SLAVE_I].rec_buff, strlen( (char *) m_buff[SLAVE_I].rec_buff), 100);
#endif
		// Some commands expect an unknown number (i.e. how many slaves are active)
		// So this command only checks if the received message CONTAINS the correct answer.
		// To make sure it checks the message verbatim, add a \n to the end of the expected answer.
		if(strstr( (char * ) m_buff[SLAVE_I].rec_buff, valid_ans) != NULL)
			return 1;
#ifdef debug
		else
			HAL_UART_Transmit(&DEBUG_UART, (uint8_t *) "Wrong Answer\n", 13, 100);
#endif
	}
#ifdef debug
	else
		HAL_UART_Transmit(&DEBUG_UART, (uint8_t *) "Timeout\n", 8, 100);
#endif

	// if the earlier return statement didn't fire, return 0, indicating either a timeout or a wrong answer
	return 0;

}

/**
 * @fn char SendMessage(const char*)
 * @brief 		Sends a message to the slaves and waits for an acknowledge.
 *
 * @pre 		none
 * @post 		Message has been sent
 * @param msg	The string to send to the slaves
 * @return		Returns 1 if acknowledge was received, else 0.
 */
char SendMessage(const char * msg)
{
#ifdef debug
	HAL_UART_Transmit(&DEBUG_UART, (uint8_t *) "Sent: " , 6			 , 100);
	HAL_UART_Transmit(&DEBUG_UART, (uint8_t *) msg	 	, strlen(msg), 100);
	strcpy( (char *) tx_debug, (char *) msg);
#else
	HAL_Delay(25);
#endif
	// Communication LED
	SetLed(BLUE_LED, LED_BLINK);

	// Every message sent to slaves requires "ACK\n" as an answer.
	HAL_UART_Transmit(&SLAVE_UART, (uint8_t *) msg, strlen(msg), 100);
	return (CheckTimeout("ACK\n", ACK_TIMEOUT) );
}

/**
 * @fn char SendCommand(const char*, const char*, int)
 * @brief 	Expanded version of the SendMessage command.
 * 			This command calls SendMessage and if that succeeded calls CheckTimout.
 * 			The user inputs what command they want to send, what answer they expect and how long to wait for a timeout.
 *
 * @pre 	none
 * @post 	Messages are sent to slaves.
 * @param cmd	The string for the command to send to the slaves.
 * @param ans	The string for the expected answer from the slaves.
 * @param timeout	How long the process will wait for the expected answer.
 * @return  1 if success, 0 if failed.
 */
char SendCommand(const char * cmd, const char * ans, int timeout)
{
	if(SendMessage(cmd) )
		if(CheckTimeout(ans, timeout) )
			return 1;
	return 0;
}

/**
 * @fn void SetLed(uint16_t, uint8_t)
 * @brief Used to control the LEDs using global defines.
 * 		  Mostly useful for the blinking functionality.
 * 		  This activates a timer that will turn the LED off after 100ms.
 * 		  This makes it so LEDs can blink without halting the execution of the program.
 *
 * @pre   none
 * @post  LED state changed
 * @param led_pin Which pin to toggle (Should use a define as input)
 * @param state What state the LED should be. On/Off/Toggle or Blink
 */
void SetLed(uint16_t led_pin, uint8_t state)
{
	switch(state)
	{
	case LED_TOGGLE:
		HAL_GPIO_TogglePin(LED_GPIO, led_pin);
		break;

	case LED_ON:
    	HAL_GPIO_WritePin(LED_GPIO, led_pin, GPIO_PIN_SET);
    	break;

	case LED_OFF:
    	HAL_GPIO_WritePin(LED_GPIO, led_pin, GPIO_PIN_RESET);
    	break;

	case LED_BLINK:
		HAL_GPIO_WritePin(LED_GPIO, led_pin, GPIO_PIN_SET);

		blink_pin = led_pin;
		__HAL_RCC_TIM2_CLK_ENABLE();
		HAL_TIM_Base_Start_IT(&LED_TIM);
		__HAL_TIM_SET_COUNTER(&LED_TIM, 0);
		break;

	default:
		break;
	}
}

/**
 * @fn char CheckMsg()
 * @brief  Checks if the screen sent a message.
 *
 * @pre 	The screen UART has to be turned on
 * @post 	none
 * @return  HALF_NUM if the message was "180\n", FULL_NUM in every other case.
 */
char CheckMsg()
{
	if(m_buff[SCHERM_I].msg_flag == 0)
		return(0);

	if(strcmp( (char *) m_buff[SCHERM_I].rec_buff, HALF) == 0)
		return(HALF_NUM);
	else
		return(FULL_NUM);
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
	  HAL_GPIO_TogglePin(LED_GPIO, BLUE_LED);
	  HAL_Delay(750);
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
