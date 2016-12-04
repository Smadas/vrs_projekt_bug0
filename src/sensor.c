/*
 * sensor.c
 *
 *  Created on: Dec 3, 2016
 *      Author: Adam Sojka
 *
 * Tato kniznica obsahuje funkcie pre inicializaciu a obsluhu troch
 * ultrazvukovych dialkomerov so spustacim impulzom a impulzom ozveny.
 */

//Includes
#include <stddef.h>
#include <string.h>
#include "stm32l1xx.h"
#include "sensor.h"

//variables
volatile uint32_t leftCaptureStep = 0;//krokovanie nabezna/dobezna hrana
volatile uint32_t rightCaptureStep = 0;
volatile uint32_t forwardCaptureStep = 0;

volatile uint16_t leftDistanceTime = 0;//konecna dlzka impulzu z dialkomeru
volatile uint16_t rightDistanceTime = 0;
volatile uint16_t forwardDistanceTime = 0;

//variables TMP
volatile uint32_t IC4ReadValue1 = 0, IC4ReadValue2 = 0;
volatile uint32_t CaptureNumber = 0;
volatile uint32_t Capture = 0;
volatile uint32_t TIM4Freq = 0;
volatile uint32_t TIM5Freq = 0;
volatile uint32_t pamatCapture[1000];
volatile uint32_t pocitadlo = 0;
volatile double pamatDist[1000];
volatile uint32_t pocitadlo1 = 0;
volatile double pamatDist1[1000];
volatile uint32_t pocitadlo11 = 0;

//defines
#define CAPTURE_CLC_PRESCALER 159
#define DISTANCE_ENV_CONST 58.0
#define DISTANCE_CLC_CONST 0.1

#define LEFT_TRIG_PIN GPIO_Pin_10
#define RIGHT_TRIG_PIN GPIO_Pin_12
#define FORWARD_TRIG_PIN GPIO_Pin_11
#define TRIG_PORT GPIOC
#define TRIG_TIM_FREQ 100000

#define STM_SYSTEM_CLOCK 16000000

//Functions
//inicializacia senzorov vzdialenosti
void sensorInit(void)
{
	sensorInitTriggerTimer();
	sensorInitTriggerPin();
	sensorInitCapturePins();
	sensorInitCaptureTimer();
}
//inicializacia casovaca, ktory generuje spustaci impulz
void sensorInitTriggerTimer(void)
{
	//vypocet delicky pre periodu 10us
	unsigned short prescalerValue = (unsigned short) (STM_SYSTEM_CLOCK / TRIG_TIM_FREQ) - 1;
	//struktura pre zakladny casovac TIM7
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//struktura pre prerusenie vyvolane TIM7
	NVIC_InitTypeDef NVIC_InitStructure;

	//spustenie hodinovych impulzov pre TIM7
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	//init struktura prerusenie
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);//zapisanie struktury

	//init struktura TIM7
	TIM_TimeBaseStructure.TIM_Period = 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

	//povolenie preruseni TIM7
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
}
//inicializacia pinov pre spustanie dialkomerov
void sensorInitTriggerPin(void)
{
	//spusti hodiny pre port C
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);//spusti hodiny pre port C

	//vytvorenie struktury GPIO
	GPIO_InitTypeDef gpioInitStruc;
	gpioInitStruc.GPIO_Mode = GPIO_Mode_OUT;
	gpioInitStruc.GPIO_OType = GPIO_OType_PP;
	gpioInitStruc.GPIO_Speed = GPIO_Speed_400KHz;

	//zapisanie inicializacnej struktury - left
	gpioInitStruc.GPIO_Pin = LEFT_TRIG_PIN;
	GPIO_Init(TRIG_PORT, &gpioInitStruc);

	//zapisanie inicializacnej struktury - right
	gpioInitStruc.GPIO_Pin = RIGHT_TRIG_PIN;
	GPIO_Init(TRIG_PORT, &gpioInitStruc);

	//zapisanie inicializacnej struktury - forward
	gpioInitStruc.GPIO_Pin = FORWARD_TRIG_PIN;
	GPIO_Init(TRIG_PORT, &gpioInitStruc);
}
//inicializacia casovaca pre meranie dlzky impulzu z dialkomera
void sensorInitCaptureTimer(void)
{
	//
	NVIC_InitTypeDef NVIC_InitStructure;
	//
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//struktura inicializacie casovaca merania dlzky impulzu TMP
	TIM_ICInitTypeDef  TIM_ICInitStructure;

	// TIM5 clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	/* TIM5 configuration: Input Capture mode ---------------------
	 The external signal is connected to TIM5 CH2 pin (PA.01)
	 The both Rising and Falling edge are used as active edge,
	 The TIM5 CCR2 is used to compute the impulse duration
	------------------------------------------------------------ */

	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;//left, forward
	TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_BothEdge;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_ICInit(TIM5, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_2;//left, forward
	TIM_ICInit(TIM5, &TIM_ICInitStructure);

	//nastavenie delicky hodinovych impuzov
	TIM_TimeBaseStructure.TIM_Prescaler = (unsigned short)CAPTURE_CLC_PRESCALER;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	// TIM enable counter
	TIM_Cmd(TIM5, ENABLE);

	// Enable the CC2 Interrupt Request
	TIM_ITConfig(TIM5, TIM_IT_CC1, ENABLE);//left
	TIM_ITConfig(TIM5, TIM_IT_CC2, ENABLE);//forward

	// Enable the TIM5 global Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
//inicializacia pinu pre meranie dlzky impulzu z dialkomera
void sensorInitCapturePins(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// GPIOB clock enable
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	// TIM5 channel 2 pin (PA.01) configuration
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
}

//spustenie merania laveho dialkomeru
void leftSensorMeasure(void)
{
	//spustenie casovaca
	TIM_Cmd(TIM7, ENABLE);
	//spustenie trig impulzu
	GPIO_WriteBit(GPIOC, LEFT_TRIG_PIN, Bit_SET);
	//nulovanie CaptureStep ak by nahodou meranie zblblo
	leftCaptureStep = 0;
}
//spustenie merania praveho dialkomeru
void rightSensorMeasure(void)
{
	//spustenie casovaca
	TIM_Cmd(TIM7, ENABLE);
	//spustenie trig impulzu
	GPIO_WriteBit(GPIOC, RIGHT_TRIG_PIN, Bit_SET);
	//nulovanie CaptureStep ak by nahodou meranie zblblo
	rightCaptureStep = 0;
}
//spustenie memrania predneho dialkomeru
void forwardSensorMeasure(void)
{
	//spustenie casovaca
	TIM_Cmd(TIM7, ENABLE);
	//spustenie trig impulzu
	GPIO_WriteBit(GPIOC, FORWARD_TRIG_PIN, Bit_SET);
	//nulovanie CaptureStep ak by nahodou meranie zblblo
	forwardCaptureStep = 0;
}

//spracovanie prerusenia z TIM7, casovac pre spustaci impulz dialkomeru
void TIM7_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) == SET)
	{
		TIM_Cmd(TIM7, DISABLE);
		if (GPIO_ReadOutputDataBit(TRIG_PORT, LEFT_TRIG_PIN) == 1)
		{
			GPIO_WriteBit(TRIG_PORT, LEFT_TRIG_PIN, Bit_RESET);//ukoncenie trig impulzu left
		}
		else if (GPIO_ReadOutputDataBit(TRIG_PORT, RIGHT_TRIG_PIN) == 1)
		{
			GPIO_WriteBit(TRIG_PORT, RIGHT_TRIG_PIN, Bit_RESET);//ukoncenie trig impulzu right
		}
		else if (GPIO_ReadOutputDataBit(TRIG_PORT, FORWARD_TRIG_PIN) == 1)
		{
			GPIO_WriteBit(TRIG_PORT, FORWARD_TRIG_PIN, Bit_RESET);//ukoncenie trig impulzu forward
		}
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	}
}

//spracovanie prerusenia z TIM5, casovac pre meranie dlzky impulzu z dialialkomerov
void TIM5_IRQHandler(void)
{
	//left
	if (TIM_GetITStatus(TIM5, TIM_IT_CC2) != RESET)
	  {


	    /* Clear TIM5 Capture compare interrupt pending bit */
	    TIM_ClearITPendingBit(TIM5, TIM_IT_CC2);
	    if(CaptureNumber == 0)
	    {
	      /* Get the Input Capture value */
	      IC4ReadValue1 = TIM_GetCapture2(TIM5);
	      CaptureNumber = 1;
	    }
	    else if(CaptureNumber == 1)
	    {
	      /* Get the Input Capture value */
	      IC4ReadValue2 = TIM_GetCapture2(TIM5);

	      /* Capture computation */
	      if (IC4ReadValue2 > IC4ReadValue1)
	      {
	        Capture = (IC4ReadValue2 - IC4ReadValue1) - 1;
	      }
	      else if (IC4ReadValue2 < IC4ReadValue1)
	      {
	        Capture = ((0xFFFF - IC4ReadValue1) + IC4ReadValue2) - 1;
	      }
	      else
	      {
	        Capture = 0;
	      }

	      /* Frequency computation */
	      TIM4Freq = Capture;
	      pamatCapture[pocitadlo] = Capture;
	      pocitadlo++;
	      if (pocitadlo >= 980)
	      {
	    	  pocitadlo = 0;
	      }
	      CaptureNumber = 0;
	    }
	  }
	//forward
	else if (TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET)
		  {


		    /* Clear TIM5 Capture compare interrupt pending bit */
		    TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);
		    if(CaptureNumber == 0)
		    {
		      /* Get the Input Capture value */
		      IC4ReadValue1 = TIM_GetCapture1(TIM5);
		      CaptureNumber = 1;
		    }
		    else if(CaptureNumber == 1)
		    {
		      /* Get the Input Capture value */
		      IC4ReadValue2 = TIM_GetCapture1(TIM5);

		      /* Capture computation */
		      if (IC4ReadValue2 > IC4ReadValue1)
		      {
		        Capture = (IC4ReadValue2 - IC4ReadValue1) - 1;
		      }
		      else if (IC4ReadValue2 < IC4ReadValue1)
		      {
		        Capture = ((0xFFFF - IC4ReadValue1) + IC4ReadValue2) - 1;
		      }
		      else
		      {
		        Capture = 0;
		      }

		      /* Frequency computation */
		      TIM5Freq = Capture;
		      //pamatCapture[pocitadlo] = Capture;
		      //pocitadlo++;
		      CaptureNumber = 0;
		    }
		  }
}

//prevzatie nameranej vzdialenosti z laveho dialkomeru
double leftSensorGetDistance(void)
{
	//vypocet vzdialenosti z laveho senzoru
	return leftDistanceTime/DISTANCE_CLC_CONST/DISTANCE_ENV_CONST;
}
//prevzatie nameranej vzdialenosti z praveho dialkomeru
double rightSensorGetDistance(void)
{
	//vypocet vzdialenosti z praveho senzoru
	return rightDistanceTime/DISTANCE_CLC_CONST/DISTANCE_ENV_CONST;
}
//prevzatie nameranej vzdialenosti z predneho dialkomeru
double forwardSensorGetDistance(void)
{
	//vypocet vzdialenosti z predneho senzoru
	return forwardDistanceTime/DISTANCE_CLC_CONST/DISTANCE_ENV_CONST;
}

