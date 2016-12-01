/*
 * dialkomer.c
 *
 *  Created on: Nov 30, 2016
 *      Author: Adam Sojka
 */


#include <stddef.h>
#include <string.h>
#include "stm32l1xx.h"
#include "dialkomer.h"

//inicializacia preriferii pre tri ultrazvukove dialkomery
int init_dialkomery()
{
	//START inicializacia pinu pre trigger
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);//spusti hodiny pre port C
	//vytvorenie struktury GPIO
	GPIO_InitTypeDef gpioInitStruc;
	gpioInitStruc.GPIO_Mode = GPIO_Mode_OUT;
	gpioInitStruc.GPIO_OType = GPIO_OType_PP;
	gpioInitStruc.GPIO_Pin = GPIO_Pin_10;
	gpioInitStruc.GPIO_Speed = GPIO_Speed_400KHz;
	//zapisanie inicializacnej struktury
	GPIO_Init(GPIOC, &gpioInitStruc);
	//END inicializacia pinu pre trigger



	return 0;
}

//urobi jedno meranie vzdialenosti ultrazvukovym snimacom
int meraj_dialkomer(int cislo_senzoru)
{
	GPIO_WriteBit(GPIOC, GPIO_Pin_10, Bit_SET);//vystup 3.3V
	GPIO_WriteBit(GPIOC, GPIO_Pin_10, Bit_RESET);//vystup 0V

	return 0;
}
