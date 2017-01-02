/*
 * kontrolka.c
 *
 *  Created on: Dec 1, 2016
 *      Author: Adam Sojka
 *
 *	Kniznica, ktora pomocou casovaca TIM6 spusta LED pripojenu na port A pin PA5.
 *	Tato LED sa kazdu sekundu vypne alebo zapne.
 */

/* Includes */
#include <stddef.h>
#include <string.h>
#include "stm32l1xx.h"
#include "runIndicatorLED.h"

/*global variables*/
long long indicatorTimeStamp;

/*defines*/
#define TIM_CLC_PRESCALER (16000000 / 1000) - 1
#define TIM_INTERRUPT_PREEMP 0
#define TIM_INTERRUPT_SUB 2
#define TIM_PERIOD 1000 - 1
#define TIM_CLC_DIV 0


/* Private function prototypes */

/* Private functions */

//inicializacia indikacnej LED chodu procesora
void init_indicator_LED()
{
	init_indicator_LED_pin();
	init_indicator_LED_trigtim();
	init_indicator_LED_trigtim_interrupt();
}

//inicializacia portu a pinu LED
void init_indicator_LED_pin()
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	GPIO_InitTypeDef gpioInitStruc;
	gpioInitStruc.GPIO_Mode = GPIO_Mode_OUT;
	gpioInitStruc.GPIO_OType = GPIO_OType_PP;
	gpioInitStruc.GPIO_Pin = GPIO_Pin_5;
	gpioInitStruc.GPIO_Speed = GPIO_Speed_400KHz;
	GPIO_Init(GPIOA, &gpioInitStruc);
}

//inicializacia casovaca pre spustanie led LED
void init_indicator_LED_trigtim()
{
	indicatorTimeStamp = 0;
	//unsigned short prescalerValue = (unsigned short) (SystemCoreClock / 1000) - 1;
	unsigned short prescalerValue = (unsigned short) TIM_CLC_PRESCALER;
	//Structure for timer settings
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	// TIM6 clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	// Enable the TIM6 gloabal Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM_INTERRUPT_PREEMP;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIM_INTERRUPT_SUB;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_TimeBaseStructure.TIM_Period = TIM_PERIOD;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CLC_DIV;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	// TIM Interrupts enable
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	// TIM6 enable counter
	TIM_Cmd(TIM6, ENABLE);
}

//inicializacia prerusenia casovaca
void init_indicator_LED_trigtim_interrupt()
{

}

//spracovanie prerusenia z TIM6, casovaca pre kontrolku
void TIM6_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) == SET)
	{
		indicatorTimeStamp++;
		GPIO_ToggleBits(GPIOA, GPIO_Pin_5);//toggle LED PA5
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
	}
}


