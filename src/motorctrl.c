/*
 * motorctrl.c
 *
 *  Created on: Dec 21, 2016
 *
 *      Author: Hombre
 *      Kniznica na ovladanie serv. Obsahuje incializaciu pinov InitializeOutput()
 *      inicializaciu timeru InitializeTimer(), dokopy v jednej funkcii Motor_init();
 *      Serva je mozne priamo ovladat pomocou prikazov left_motor_set_speed a right_motor_set_speed,
 *      pricom parameter je cislo v rozsahu -20-20 pricom -20 naplno dozadu 0 stoji 20 naplno dopredu
 */
#include <stddef.h>
#include "stm32l1xx.h"
#include <misc.h>

#define MID_SERVO_VALUE 150
#define TIM_PERIOD 1000
#define SYS_FREQ 16000000
#define PRESC 100000
#define UP_LIMIT 20
#define LOW_LIMIT -20

int volatile counter=0;
int volatile counter2=0;


TIM_OCInitTypeDef  TIM_OCInitStructure;
void InitializeOutput()
{GPIO_InitTypeDef GPIO_InitStructure;

		  /* TIM3 clock enable */
		  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		  /* GPIOA and GPIOB clock enable */
		  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);


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

	uint16_t pwmvalue = MID_SERVO_VALUE;
	uint16_t pwmvalueL = MID_SERVO_VALUE;
	uint16_t myperiod = TIM_PERIOD;
	uint16_t PrescalerValue = 0;

	PrescalerValue = (uint16_t)(SYS_FREQ/PRESC) - 1;
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

	if(a>UP_LIMIT){
		a=UP_LIMIT;
	}
	if(a<-LOW_LIMIT){
		a=-LOW_LIMIT;
	}

	if (a!=counter){
		TIM_OCInitStructure.TIM_Pulse = MID_SERVO_VALUE-a;
		TIM_OC3Init(TIM2, &TIM_OCInitStructure);//pravy pwm update
		counter=a;
	}

}

void left_motor_set_speed(int a){

	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	if(a>UP_LIMIT){
		a=UP_LIMIT;
	}
	if(a<-LOW_LIMIT){
		a=-LOW_LIMIT;
	}
	if (a!=counter2){
	 TIM_OCInitStructure.TIM_Pulse = MID_SERVO_VALUE+a;
	 TIM_OC4Init(TIM2, &TIM_OCInitStructure);//lavy pwm update
	 counter2=a;
	}


}

