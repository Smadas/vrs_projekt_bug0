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

void initUSART2();
void PutcUART2(char ch);
void USART2_IRQHandler(void);

void sendValue();


#endif /* BLUETOOTH_H_ */