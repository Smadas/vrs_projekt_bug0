/*
 * bluetooth.h
 *
 *  Created on: 11. 12. 2016
 *      Author: michal1
 */

#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

#define GOAL_RANGE 8

#include <stddef.h>
#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"


int running;
int change_goal_request;
int goal_bearing;

void initUSART3();
void PutcUART3(char ch);
void USART3_IRQHandler(void);

void sendValue(double prekazka);


#endif /* BLUETOOTH_H_ */
