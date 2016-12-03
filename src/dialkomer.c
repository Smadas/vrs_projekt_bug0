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

//Private variables
volatile uint32_t IC4ReadValue1 = 0, IC4ReadValue2 = 0;
volatile uint32_t CaptureNumber = 0;
volatile uint32_t Capture = 0;
volatile uint32_t TIM4Freq = 0;
volatile uint32_t pamatCapture[100];
volatile uint32_t pocitadlo = 0;
volatile double pamatDist[100];
volatile uint32_t pocitadlo1 = 0;

//Private typedef
TIM_ICInitTypeDef  TIM_ICInitStructure;

//inicializacia preriferii pre tri ultrazvukove dialkomery
int init_dialkomery()
{
	int myErr = 0;

	//inicializacia pinu pre spustac dialkomeru
	if (init_spustac_dialkomer() == -1)
	{
		//error nepodarila sa inicializacia pinu pre spustac dialkomeru
		myErr = -1;
	}

	//inicializacia casovaca capture pre meranie velkosti echo impulzu z dialkomeru
	if (init_cas_zachyt_imp_dialkomer() == -1)
	{
		//error nepodarila sa inicializacia casovaca capture pre meranie velkosti echo impulzu z dialkomeru
		myErr = -1;
	}

	//inicializacia casovaca pre spustaci impulz dialkomera
	if (init_cas_trig_dialkomer() == -1)
	{
		//error nepodarila sa inicializacia casovaca pre spustaci impulz dialkomera
		myErr = -1;
	}

	return myErr;
}

//inicializacia pinu pre spustac dialkomeru
int init_spustac_dialkomer()
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

	return 0;
}

//
int init_cas_zachyt_imp_dialkomer()
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

/*
	  // TIM4 clock enable
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	  // GPIOB clock enable
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	  // TIM4 channel 2 pin (PB.07) configuration
	  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
*/
	  /* TIM4 configuration: Input Capture mode ---------------------
	     The external signal is connected to TIM4 CH2 pin (PB.07)
	     The Rising edge is used as active edge,
	     The TIM4 CCR2 is used to compute the frequency value
	  ------------------------------------------------------------ */
/*
	  TIM_ICInitStructure.TIM_Channel     = TIM_Channel_2;
	  TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_BothEdge;//TIM_ICPolarity_Rising;
	  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	  TIM_ICInitStructure.TIM_ICFilter = 0x0;
	  TIM_ICInit(TIM4, &TIM_ICInitStructure);

	  // TIM enable counter
	  TIM_Cmd(TIM4, ENABLE);

	  // Enable the CC2 Interrupt Request
	  TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);

	  // Enable the TIM4 global Interrupt
	  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
*/
	  /*
	  // TIM4 clock enable
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	  //GPIOB clock enable
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	  // TIM2 channel 1 pin (PA.07) configuration
	  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM2);
*/
	  /* TIM2 configuration: Input Capture mode ---------------------
	     The external signal is connected to TIM2 CH2 pin (PA.00)
	     The Rising edge is used as active edge,
	     The TIM4 CCR2 is used to compute the frequency value
	  ------------------------------------------------------------ */
/*
	  TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;
	  TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
	  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	  TIM_ICInitStructure.TIM_ICFilter = 0x0;
	  TIM_ICInit(TIM2, &TIM_ICInitStructure);

	  // TIM enable counter
	  TIM_Cmd(TIM2, ENABLE);

	  // Enable the CC2 Interrupt Request
	  TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);

	  // Enable the TIM4 global Interrupt
	  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
	  */

	return 0;
}

//spracovanie prerusenia z TIM7, casovac pre spustanie dialkomeru
void TIM7_IRQHandler(void)
{
	TIM_Cmd(TIM7, DISABLE);
	GPIO_WriteBit(GPIOC, GPIO_Pin_10, Bit_RESET);//ukoncenie trig impulzu
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	}
}

//urobi jedno meranie vzdialenosti ultrazvukovym snimacom
//bude sa spustat s casovacom TIM6, ktory je teraz pouzity
//na kontrolku ale to sa upravi aby tikal s frekvenciou 10Hz
//teda sa meranie bude robit kazdych 100ms
int meraj_dialkomer(int cislo_senzoru)
{
	TIM_Cmd(TIM7, ENABLE);
	GPIO_WriteBit(GPIOC, GPIO_Pin_10, Bit_SET);//spustenie trig impulzu

	return 0;
}

double citaj_vzdialenost(int cislo_senzoru)
{
	pocitadlo1++;
	pamatDist[pocitadlo1] = TIM4Freq/16.0/58.0;
	return TIM4Freq/16.0/58.0;
}

//inicializacia casovaca pre spustaci impulz dialkomeru
int init_cas_trig_dialkomer()
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

	return 0;
}

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
  /*if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)
  {
    // Clear TIM2 Capture compare interrupt pending bit
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
    Capture = TIM_GetCapture2(TIM4);
    if(CaptureNumber == 0)
    {
      // Get the Input Capture value
      IC4ReadValue1 = TIM_GetCapture1(TIM2);
      CaptureNumber = 1;
    }
    else if(CaptureNumber == 1)
    {

      // Get the Input Capture value
      IC4ReadValue2 = TIM_GetCapture1(TIM2);

      // Capture computation
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
    	Capture =
      // Frequency computation
      TIM2Freq = (uint32_t) 16000000 / Capture;
      CaptureNumber = 0;
    }
  }*/
}
