/*
 * kontrolka.c
 *
 *  Created on: Dec 1, 2016
 *      Author: Adam Sojka
 *
 */

/* Includes */
#include <stddef.h>
#include <string.h>
#include "stm32l1xx.h"
#include "dialkomer.h"

/*global variables*/
long long gTimeStamp;

/* Private function prototypes */

/* Private functions */
int init_kontrolka()
{
	//spustenie hodin pre periferiu
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	//vytvorenie struktury GPIO
	GPIO_InitTypeDef gpioInitStruc;
	gpioInitStruc.GPIO_Mode = GPIO_Mode_OUT;
	gpioInitStruc.GPIO_OType = GPIO_OType_PP;
	gpioInitStruc.GPIO_Pin = GPIO_Pin_5;
	gpioInitStruc.GPIO_Speed = GPIO_Speed_400KHz;
	//zapisanie inicializacnej struktury
	GPIO_Init(GPIOA, &gpioInitStruc);

	return 0;
}

int init_cas_blikanie()
{
	gTimeStamp = 0;
	//unsigned short prescalerValue = (unsigned short) (SystemCoreClock / 1000) - 1;
	unsigned short prescalerValue = (unsigned short) (16000000 / 1000) - 1;
	/*Structure for timer settings*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	/* TIM6 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	/* Enable the TIM6 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_TimeBaseStructure.TIM_Period = 1000;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	/* TIM Interrupts enable */
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	/* TIM6 enable counter */
	TIM_Cmd(TIM6, ENABLE);

	return 0;
}

void TIM6_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) == SET)
	{
		gTimeStamp++;
		GPIO_ToggleBits(GPIOA, GPIO_Pin_5);//toggle LED PA5
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
	}
}


