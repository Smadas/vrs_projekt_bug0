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
#define DISTANCE_MAX 500

	//trig
#define LEFT_TRIG_PIN GPIO_Pin_10
#define RIGHT_TRIG_PIN GPIO_Pin_12
#define FORWARD_TRIG_PIN GPIO_Pin_11
#define TRIG_PORT GPIOC
#define TRIG_TIM_FREQ 100000

	//capture
#define LEFT_TIM_CHANNEL TIM_Channel_3
#define RIGHT_TIM_CHANNEL TIM_Channel_1
#define FORWARD_TIM_CHANNEL TIM_Channel_4
#define LEFT_TIM_CC TIM_IT_CC3
#define RIGHT_TIM_CC TIM_IT_CC1
#define FORWARD_TIM_CC TIM_IT_CC4
#define LEFT_CAP_PIN GPIO_Pin_0
#define RIGHT_CAP_PIN GPIO_Pin_4
#define FORWARD_CAP_PIN GPIO_Pin_1
#define CAPTURE_PORT GPIOB
#define LEFT_CAP_PINSOURCE GPIO_PinSource0
#define RIGHT_CAP_PINSOURCE GPIO_PinSource4
#define FORWARD_CAP_PINSOURCE GPIO_PinSource1

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

	//spustenie hodinovych impulzov pre TIM7
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	//TIM7 init prerusenie
	sensorInitTriggerTimerInterrup();

	//init struktura TIM7
	TIM_TimeBaseStructure.TIM_Period = 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);//zapisanie struktury

	//povolenie preruseni TIM7
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
}
//inicializacia preruseni casovaca, ktory generuje spustaci impulz
void sensorInitTriggerTimerInterrup(void)
{
	//TIM7 struct prerusenie
	NVIC_InitTypeDef NVIC_InitStructure;

	//init struktura prerusenie
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);//zapisanie struktury
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
	//TIM3 struct zaklad
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//TIM3 struct capture
	TIM_ICInitTypeDef  TIM_ICInitStructure;

	//povolenie hodin pre TIM3
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	//init TIM3 struct capture
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;

	//TIM3 nastavenie kanalov
	TIM_ICInitStructure.TIM_Channel = LEFT_TIM_CHANNEL;//left
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = RIGHT_TIM_CHANNEL;//right
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = FORWARD_TIM_CHANNEL;//forward
	TIM_ICInit(TIM3, &TIM_ICInitStructure);

	//nastavenie delicky hodinovych impuzov
	TIM_TimeBaseStructure.TIM_Prescaler = (unsigned short)CAPTURE_CLC_PRESCALER;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	//TIM3 povolenie pocitadla
	TIM_Cmd(TIM3, ENABLE);

	//povolenie CC poziadavky na prerusenie
	TIM_ITConfig(TIM3, LEFT_TIM_CC, ENABLE);//left
	TIM_ITConfig(TIM3, RIGHT_TIM_CC, ENABLE);//right
	TIM_ITConfig(TIM3, FORWARD_TIM_CC, ENABLE);//forward

	//TIM3 prerusenie init
	sensorInitCaptureTimerInterrup();

	//mozno to tam chyba TEMP
	//povolenie preruseni TIM7
	//TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}
//inicializacia preruseni casovaca pre meranie dlzky impulzu z dialkomera
void sensorInitCaptureTimerInterrup(void)
{
	//TIM3 struct prerusenie
	NVIC_InitTypeDef NVIC_InitStructure;

	//init TIM struct prerusenie
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
//inicializacia pinu pre meranie dlzky impulzu z dialkomera
void sensorInitCapturePins(void)
{
	//GPIO struct
	GPIO_InitTypeDef GPIO_InitStructure;

	//GPIOB povolenie hodin
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	//TIM3 struct init
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

	//inicializacia pinu left
	GPIO_InitStructure.GPIO_Pin   = LEFT_CAP_PIN;
	GPIO_Init(CAPTURE_PORT, &GPIO_InitStructure);//zapisanie struktury
	GPIO_PinAFConfig(CAPTURE_PORT, LEFT_CAP_PINSOURCE, GPIO_AF_TIM3);
	//inicializacia pinu right
	GPIO_InitStructure.GPIO_Pin   = RIGHT_CAP_PIN;
	GPIO_Init(CAPTURE_PORT, &GPIO_InitStructure);//zapisanie struktury
	GPIO_PinAFConfig(CAPTURE_PORT, RIGHT_CAP_PINSOURCE, GPIO_AF_TIM3);
	//inicializacia pinu forward
	GPIO_InitStructure.GPIO_Pin   = FORWARD_CAP_PIN;
	GPIO_Init(CAPTURE_PORT, &GPIO_InitStructure);//zapisanie struktury
	GPIO_PinAFConfig(CAPTURE_PORT, FORWARD_CAP_PINSOURCE, GPIO_AF_TIM3);
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
		//vypnutie impulzu na prislusnom spustacom pine
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
void TIM3_IRQHandler(void)
{
	//rozdelit do troch funkcii
	//este treba spravit spustanie dialkomerov cez casovac
	//left
	if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
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
	double distance = leftDistanceTime/DISTANCE_CLC_CONST/DISTANCE_ENV_CONST;
	if (distance > DISTANCE_MAX)
	{
		//senzor nic nezachytil
		return -1;
	}
	else
	{
		return distance;
	}
}
//prevzatie nameranej vzdialenosti z praveho dialkomeru
double rightSensorGetDistance(void)
{
	//vypocet vzdialenosti z praveho senzoru
	double distance = rightDistanceTime/DISTANCE_CLC_CONST/DISTANCE_ENV_CONST;
	if (distance > DISTANCE_MAX)
	{
		//senzor nic nezachytil
		return -1;
	}
	else
	{
		return distance;
	}
}
//prevzatie nameranej vzdialenosti z predneho dialkomeru
double forwardSensorGetDistance(void)
{
	//vypocet vzdialenosti z predneho senzoru
	double distance = forwardDistanceTime/DISTANCE_CLC_CONST/DISTANCE_ENV_CONST;
	if (distance > DISTANCE_MAX)
	{
		//senzor nic nezachytil
		return -1;
	}
	else
	{
		return distance;
	}
}

