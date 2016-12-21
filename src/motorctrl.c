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
int stat=1;
int stat2=1;
int LM=74;
int PM=74;

void InitializeLEDs()
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_InitTypeDef gpioInit;
     gpioInit.GPIO_Mode = GPIO_Mode_OUT;
     gpioInit.GPIO_OType = GPIO_OType_PP;
     gpioInit.GPIO_Pin =  GPIO_Pin_6 ;
     gpioInit.GPIO_Speed = GPIO_Speed_40MHz;
     GPIO_Init(GPIOB, &gpioInit);

     gpioInit.GPIO_Mode = GPIO_Mode_OUT;
          gpioInit.GPIO_OType = GPIO_OType_PP;
          gpioInit.GPIO_Pin = GPIO_Pin_7 ;
          gpioInit.GPIO_Speed = GPIO_Speed_40MHz;
          GPIO_Init(GPIOA, &gpioInit);

    GPIO_WriteBit(GPIOB, GPIO_Pin_6, Bit_SET);
}

void InitializeTimer()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef timerInitStructure;
  timerInitStructure.TIM_Prescaler = 9;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = 15;
    timerInitStructure.TIM_ClockDivision = 0;
  //  timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &timerInitStructure);
    TIM_Cmd(TIM4, ENABLE);

    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
}
void EnableTimerInterrupt()
{
    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIM4_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
    nvicStructure.NVIC_IRQChannelSubPriority = 1;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);
   // TIM4->DIER = TIM_DIER_UIE;
}

void right_motor_set_speed(int a){

	if(a>5){a=5;}
	if(a<-5){a=-5;}
	PM=74-a;
}

void left_motor_set_speed(int a){
	if(a>5){a=5;}
	if(a<-5){a=-5;}
	LM=74+a;
}

void TIM4_IRQHandler()
{


    	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
    		{


    		counter++;
    	    	counter2++;


       if(stat==1){

        if(counter>=LM) {
        	stat=0;
        	GPIO_ResetBits(GPIOB, GPIO_Pin_6);
        	counter=0;
        }

       }
        else {

        	if(counter>=2000) {
        		stat=1;GPIO_SetBits(GPIOB, GPIO_Pin_6);
        		counter=0;
        	}
        }

      if(stat2==1){

                if(counter2>=PM) {
                	stat2=0;
                	GPIO_ResetBits(GPIOA, GPIO_Pin_7);
                	counter2=0;
                }
      }
       else {

             if(counter2>=2000) {
            	 stat2=1;
            	 GPIO_SetBits(GPIOA, GPIO_Pin_7);
            	 counter2=0;
             }
       }






      TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}

