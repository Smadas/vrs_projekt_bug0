/*
 * motorctrl.c
 *
 *  Created on: Dec 21, 2016
 *      Author: Hombre
 */
#include <stddef.h>
#include "stm32l1xx.h"
#include <misc.h>


int volatile counter=0;
int volatile counter2=0;
//int volatile counter3=0;

TIM_OCInitTypeDef  TIM_OCInitStructure;
void InitializeOutput()
{GPIO_InitTypeDef GPIO_InitStructure;
	/* --------------------------- System Clocks Configuration ---------------------*/
		  /* TIM3 clock enable */
		  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		  /* GPIOA and GPIOB clock enable */
		  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

		  /*--------------------------------- GPIO Configuration -------------------------*/

		  /* GPIOB Configuration: Pin 10 and 11 */
		  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10|GPIO_Pin_11;
		  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
		  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;

		  GPIO_Init(GPIOB, &GPIO_InitStructure);
		  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_TIM2);
		  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_TIM2);
}

void InitializeTimer()
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	uint16_t pwmvalue = 150;
	uint16_t pwmvalueL = 150;
	uint16_t myperiod = 1000;
	uint16_t PrescalerValue = 0;

	 PrescalerValue = (uint16_t)(16000000/100000) - 1;
		  /* Time base configuration */
		  TIM_TimeBaseStructure.TIM_Period = myperiod - 1;
		  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
		  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

		  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

		  /* PWM1 Mode configuration: Channel3 */
		  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

		  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		  TIM_OCInitStructure.TIM_Pulse = pwmvalue;
		  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;


		  TIM_OC3Init(TIM2, &TIM_OCInitStructure);//pravy

		  TIM_OCInitStructure.TIM_Pulse = pwmvalueL;//lavy
		  TIM_OC4Init(TIM2, &TIM_OCInitStructure);


		  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
		  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);



		  /* TIM3 enable counter */
		  TIM_Cmd(TIM2, ENABLE);

}
void Motor_init()
{
	InitializeOutput();
InitializeTimer();

}

void right_motor_set_speed(int a){
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

			  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

			  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	if(a>20){a=20;}
	if(a<-20){a=-20;}

	  TIM_OCInitStructure.TIM_Pulse = 150-a;
	  TIM_OC3Init(TIM2, &TIM_OCInitStructure);//pravy pwm update
}

void left_motor_set_speed(int a){
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

			  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

			  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	if(a>20){a=20;}
	if(a<-20){a=-20;}
	 TIM_OCInitStructure.TIM_Pulse = 150+a;
	 TIM_OC4Init(TIM2, &TIM_OCInitStructure);//lavy pwm update
}

