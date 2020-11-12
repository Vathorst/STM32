/*
 * buttons.h
 *
 *  Created on: 6 Oct 2020
 *      Author: Jeroen
 */

#ifndef INC_BUTTONS_H_
#define INC_BUTTONS_H_
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define BUTTON_CHOOSE_METHOD_CHANCE 0
#define BUTTON_CHOOSE_METHOD_SEQUENCE 1
#define BUTTON_CHOOSE_METHOD_ALGORITHM 2

#define BUTTON_CHOOSE_ERR -1

#define BUTTON_AMOUNT 20 //TODO: Make this variable

char chooseButton(char c_method);
char signalButton(char c_button);

#endif /* INC_BUTTONS_H_ */
