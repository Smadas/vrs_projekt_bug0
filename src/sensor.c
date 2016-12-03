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

}
//inicializacia pinov pre spustanie dialkomerov
void sensorInitTriggerPin(void);
//inicializacia casovaca pre meranie dlzky impulzu z dialkomera
void sensorInitCaptureTimer(void);
//inicializacia pinu pre meranie dlzky impulzu z dialkomera
void sensorInitCapturePins(void);

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
double leftSensorGetDistance(void);
//prevzatie nameranej vzdialenosti z praveho dialkomeru
double rightSensorGetDistance(void);
//prevzatie nameranej vzdialenosti z predneho dialkomeru
double forwardSensorGetDistance(void);

