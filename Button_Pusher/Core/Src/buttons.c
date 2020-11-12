/*
 * buttons.c
 *
 *  Created on: 6 Oct 2020
 *      Author: Jeroen
 */
#include "main.h"

/**
  * @brief Button Function for Choosing a Button, based on Chance, a predefined Sequence or an Algorithm
  * @param None
  * @retval Chosen Button
  */

/**
  * @brief Button Function for Choosing a Button, based on Chance, a predefined Sequence or an Algorithm
  * @param None
  * @retval Chosen Button
  */
char chooseButton(char c_method)
{
	char chosen_button = 0;
	srand(time(0));
	switch (c_method){
		case BUTTON_CHOOSE_METHOD_CHANCE:
			 chosen_button = rand()%BUTTON_AMOUNT; //TODO: Test function
			 break;
		case BUTTON_CHOOSE_METHOD_SEQUENCE: //TODO: Create Sequence(s) of numbers. Requires knowing button location
			 break;
		case BUTTON_CHOOSE_METHOD_ALGORITHM: //...//
			 break;
		default: chosen_button = BUTTON_CHOOSE_ERR;
			 break;
	}
	return chosen_button;
}

/**
  * @brief Will signal the chosen button to Activate.
  * @param None
  * @retval Active Button
  */
char signalButton(char c_button)
{
	unsigned char buf[3];

	//Byte of 1 means activate button (Temporarily until actual commands are known)
	buf[0] = 1;

	//Returns status and stores it into ret
	char ret;
 	ret = HAL_I2C_Master_Transmit(&hi2c1, 55, (uint8_t *) buf, 1, HAL_MAX_DELAY); //55 for adress is placeholder until actual adresses are known
 	if(ret != HAL_OK)
 	{
 		HAL_Delay(1000);
 		//Delay is temp, prevents spamming of main loop.
 	}
 	return 0;
}
