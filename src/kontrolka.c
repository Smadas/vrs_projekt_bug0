/*
 * kontrolka.c
 *
 *  Created on: Dec 1, 2016
 *      Author: Adam Sojka
 *
 */

#include <stddef.h>
#include <string.h>
#include "stm32l1xx.h"
#include "dialkomer.h"

void (*gBaseTimerHandler)(long long timeStamp);

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

void blik_kontrolka()
{
	GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);//zapne led
	for(int i = 0; i < 100000; i++)
	{

	}
	GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);//vypne led
	for(int i = 0; i < 100000; i++)
	{

	}
}

int init_cas_blikanie()
{
	gTimeStamp = 0;
	unsigned short prescalerValue = (unsigned short) (SystemCoreClock / 1000) - 1;
	/*Structure for timer settings*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	/* TIM2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	/* Enable the TIM3 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_TimeBaseStructure.TIM_Period = 1000;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	/* TIM Interrupts enable */
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	/* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);

	return 0;
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		if (gBaseTimerHandler != 0)
		{
			gTimeStamp++;
			gBaseTimerHandler(gTimeStamp);
		}
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}

void registerBaseTimerHandler(void (*handler)(long long timeStamp))
{
	gBaseTimerHandler = handler;
}

