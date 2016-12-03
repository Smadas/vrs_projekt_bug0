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

//Private variables TMP
volatile uint32_t IC4ReadValue1 = 0, IC4ReadValue2 = 0;
volatile uint32_t CaptureNumber = 0;
volatile uint32_t Capture = 0;
volatile uint32_t TIM4Freq = 0;
volatile uint32_t pamatCapture[100];
volatile uint32_t pocitadlo = 0;
volatile double pamatDist[100];
volatile uint32_t pocitadlo1 = 0;

//Private typedef
TIM_ICInitTypeDef  TIM_ICInitStructure;//struktura inicializacie casovaca merania dlzky impulzu TMP


//Functions
//inicializacia senzorov vzdialenosti
void sensorInit(void)
{
	sensorInitTriggerTimer();
	sensorInitTriggerPin();
	sensorInitCaptureTimer();
	sensorInitCapturePins();
}
//inicializacia casovaca, ktory generuje spustaci impulz
void sensorInitTriggerTimer(void)
{
	//unsigned short prescalerValue = (unsigned short) (SystemCoreClock / 1000) - 1;
	unsigned short prescalerValue = (unsigned short) (16000000 / 100000) - 1;
	//Structure for timer settings
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	// TIM6 clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	// Enable the TIM7 gloabal Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_TimeBaseStructure.TIM_Period = 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	// TIM Interrupts enable
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	// TIM7 enable counter
	//TIM_Cmd(TIM7, ENABLE);
}
//inicializacia pinov pre spustanie dialkomerov
void sensorInitTriggerPin(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);//spusti hodiny pre port C
	//vytvorenie struktury GPIO
	GPIO_InitTypeDef gpioInitStruc;
	gpioInitStruc.GPIO_Mode = GPIO_Mode_OUT;
	gpioInitStruc.GPIO_OType = GPIO_OType_PP;
	gpioInitStruc.GPIO_Pin = GPIO_Pin_10;
	gpioInitStruc.GPIO_Speed = GPIO_Speed_400KHz;
	//zapisanie inicializacnej struktury
	GPIO_Init(GPIOC, &gpioInitStruc);
}
//inicializacia casovaca pre meranie dlzky impulzu z dialkomera
void sensorInitCaptureTimer(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;

	  // TIM5 clock enable
	  	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

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

	  	  /* TIM5 configuration: Input Capture mode ---------------------
	  	     The external signal is connected to TIM5 CH2 pin (PA.01)
	  	     The Rising edge is used as active edge,
	  	     The TIM5 CCR2 is used to compute the frequency value
	  	  ------------------------------------------------------------ */

	  	  TIM_ICInitStructure.TIM_Channel     = TIM_Channel_2;
	  	  TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_BothEdge;//TIM_ICPolarity_Rising;
	  	  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	  	  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	  	  TIM_ICInitStructure.TIM_ICFilter = 0x0;
	  	  TIM_ICInit(TIM5, &TIM_ICInitStructure);

	  	  // TIM enable counter
	  	  TIM_Cmd(TIM5, ENABLE);

	  	  // Enable the CC2 Interrupt Request
	  	  TIM_ITConfig(TIM5, TIM_IT_CC2, ENABLE);

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

}

//spustenie merania laveho dialkomeru
void leftSensorMeasure(void)
{
	TIM_Cmd(TIM7, ENABLE);
	GPIO_WriteBit(GPIOC, GPIO_Pin_10, Bit_SET);//spustenie trig impulzu
}
//spustenie merania praveho dialkomeru
void rightSensorMeasure(void)
{

}
//spustenie memrania predneho dialkomeru
void forwardSensorMeasure(void)
{

}

//spracovanie prerusenia z TIM7, casovac pre spustaci impulz dialkomeru
void TIM7_IRQHandler(void)
{
	TIM_Cmd(TIM7, DISABLE);
	GPIO_WriteBit(GPIOC, GPIO_Pin_10, Bit_RESET);//ukoncenie trig impulzu
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	}
}

//spracovanie prerusenia z TIM5, casovac pre meranie dlzky impulzu z dialialkomerov
void TIM5_IRQHandler(void)
{
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
	      CaptureNumber = 0;
	    }
	  }
}

//prevzatie nameranej vzdialenosti z laveho dialkomeru
double leftSensorGetDistance(void)
{
	pocitadlo1++;
	pamatDist[pocitadlo1] = TIM4Freq/16.0/58.0;
	return TIM4Freq/16.0/58.0;
}
//prevzatie nameranej vzdialenosti z praveho dialkomeru
double rightSensorGetDistance(void)
{
	return 0;
}
//prevzatie nameranej vzdialenosti z predneho dialkomeru
double forwardSensorGetDistance(void)
{
	return 0;
}

