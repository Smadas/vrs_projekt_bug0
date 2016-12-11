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

#define PRN_USART USART2
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



void USART2_IRQHandler(void)
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
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; //zoznam prerušení nájdete v súbore stm32l1xx.h
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}
void inicializaciaUSART2(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);


	/* Configure USART Tx and Rx pins */
	GPIO_InitTypeDef GPIO_InitStructure;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  //usart configuration
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	  USART_InitTypeDef USART_InitStructure;
	  USART_InitStructure.USART_BaudRate = 19200;
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;
	  USART_InitStructure.USART_Parity = USART_Parity_No;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	  USART_Init(USART2, &USART_InitStructure);
	  USART_Cmd(USART2, ENABLE);
	  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	  USART_ITConfig(USART2, USART_IT_TC, ENABLE);
}




