/*
 * serial.c
 *
 *  Created on: Dec 11, 2016
 *      Author: Adam Sojka
 */


#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include "stm32l1xx.h"
#include "serial.h"

#define PRE_JOIN_DEF(X,Y,Z) X##Y##Z
#define JOIN_DEF(X,Y,Z) PRE_JOIN_DEF(X,Y,Z)

//USART PRINT TO CONSOLE
#define PRN_USART_NUM 3//3//2
#define PRN_USART_TXDPIN 10//10//2
#define PRN_USART_RXDPIN 11//11//3
#define PRN_USART_GPIOPORT_LETTER B//B//A
#define PRN_USART_RCC_NUM 1//1//1

#define PRN_USART_GPIOPORT JOIN_DEF(GPIO,PRN_USART_GPIOPORT_LETTER, )
#define PRN_USART_TXDPIN_SOURCE JOIN_DEF(GPIO_PinSource,PRN_USART_TXDPIN, )
#define PRN_USART_RXDPIN_SOURCE JOIN_DEF(GPIO_PinSource,PRN_USART_RXDPIN, )
#define PRN_USART_TXDPIN_PIN JOIN_DEF(GPIO_Pin_,PRN_USART_TXDPIN, )
#define PRN_USART_RXDPIN_PIN JOIN_DEF(GPIO_Pin_,PRN_USART_RXDPIN, )
#define PRN_USART_GPIO_RCC JOIN_DEF(RCC_AHBPeriph_GPIO,PRN_USART_GPIOPORT_LETTER, )
#define PRN_USART_RCC_FUNC JOIN_DEF(RCC_APB,PRN_USART_RCC_NUM,PeriphClockCmd)
#define PRN_USART_RCC_ARG JOIN_DEF(RCC_APB,JOIN_DEF(PRN_USART_RCC_NUM,Periph_USART,PRN_USART_NUM), )
#define PRN_USART_NVIC_IRQ JOIN_DEF(USART,PRN_USART_NUM,_IRQn)

#define PRN_USART JOIN_DEF(USART,PRN_USART_NUM, )
#define PRN_USART_IRQHANDLER JOIN_DEF(USART,PRN_USART_NUM,_IRQHandler)
#define PRN_USART_GPIO_AF JOIN_DEF(GPIO_AF_USART,PRN_USART_NUM, )

//GPIO_AF_USART2
#define NUM_DEC_CONST 100

volatile int USARTbufferRDY;
volatile char USARTbufferOut[100]; //buffer, kde su ulozene znaky na odoslanie
volatile int USARTbufferInkr;

//ulozenie spravy do buffera pre USART a odosle prvy znak
int printToUSARTbuffer(char *message)
{
	if (USARTbufferRDY == 1)
	{
		int increment = 0;
		while (message[increment] != 0)
		{
			USARTbufferOut[increment] = message[increment];
			increment++;
		}
		USARTbufferInkr = 0;
		if (USARTbufferOut[USARTbufferInkr] != 0)
		{
			USART_SendData(PRN_USART, USARTbufferOut[USARTbufferInkr]);
		}
		else
		{
			return -2;//chyba prazdna sprava
		}

	}
	else
	{
		return -1;//chyba USART buffer nie je pripraveny
	}

	return 0;
}
//dialkomer - vytvorenie spravy so vzdialenostou
int sensorMessage(double distance, int sensorNum)
{
	char message[100];

	//rozdelenie vzdialenosti na celu a desatinnu cast
	int decimalPart = 0;
	int integerPart = 0;
	integerPart = (int)distance;
	decimalPart = (int)(distance*NUM_DEC_CONST) - integerPart*NUM_DEC_CONST;
	//osetrenie prichodu -1 - teda mimo rozsah
	if (distance < 0)
	{
		decimalPart = 0;
		integerPart = -1;
	}

	sprintf(message, "S: %d vzd: %d.%d", sensorNum, integerPart, decimalPart);

	//pridanie noveho riadku
	int increment = 0;
	while (message[increment] != 0)
	{
		//hladanie konca spravy
		increment++;
	}
	message[increment + 1] = 13;//carriage return
	message[increment] = 10;//new line
	message[increment + 2] = 0;

	//odoslanie prveho znaku
	switch (printToUSARTbuffer(message))
	{
	case -1:
		return -1;//USART buffer nie je pripraveny
		break;
	case -2:
		return -2;//prazdna sprava
		break;
	}

	return 0;
}



void PRN_USART_IRQHANDLER(void)
{
	if(USART_GetITStatus(PRN_USART, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(PRN_USART, USART_IT_RXNE);
		receivedChar = USART_ReceiveData(PRN_USART);
	}
	if(USART_GetFlagStatus(PRN_USART, USART_FLAG_TC) != RESET)
	{
		USART_ClearFlag(PRN_USART, USART_FLAG_TC);
		USARTbufferInkr++;
		if( USARTbufferOut[USARTbufferInkr] != '\0')
		{
			USART_SendData(PRN_USART, USARTbufferOut[USARTbufferInkr]);
		}
		else
		{
			USARTbufferInkr = 0;
			USARTbufferRDY = 1;
		}
	}
}


void inicializaciaPrerusenieUSART(void)
{
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = PRN_USART_NVIC_IRQ ; //zoznam prerušení nájdete v súbore stm32l1xx.h
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}
void inicializaciaUSART(void)
{
	RCC_AHBPeriphClockCmd(PRN_USART_GPIO_RCC, ENABLE);

	/* Configure USART Tx and Rx pins */
	GPIO_InitTypeDef GPIO_InitStructure;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Pin = PRN_USART_RXDPIN_PIN | PRN_USART_TXDPIN_PIN;
	  GPIO_PinAFConfig(PRN_USART_GPIOPORT, PRN_USART_TXDPIN_SOURCE, PRN_USART_GPIO_AF);
	  GPIO_PinAFConfig(PRN_USART_GPIOPORT, PRN_USART_RXDPIN_SOURCE, PRN_USART_GPIO_AF);
	  GPIO_Init(PRN_USART_GPIOPORT, &GPIO_InitStructure);

	  //usart configuration
	  PRN_USART_RCC_FUNC(PRN_USART_RCC_ARG, ENABLE);

	  USART_InitTypeDef USART_InitStructure;
	  USART_InitStructure.USART_BaudRate = 19200;
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;
	  USART_InitStructure.USART_Parity = USART_Parity_No;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	  USART_Init(PRN_USART, &USART_InitStructure);
	  USART_Cmd(PRN_USART, ENABLE);
	  USART_ITConfig(PRN_USART, USART_IT_RXNE, ENABLE);
	  USART_ITConfig(PRN_USART, USART_IT_TC, ENABLE);
}




