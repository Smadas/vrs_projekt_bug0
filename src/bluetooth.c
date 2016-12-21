/*
 * bluetooth.c
 *
 *  Created on: 11. 12. 2016
 *      Author: michal1
 */


#include <bluetooth.h>

void initUSART2(){

	running = 0;
	change_goal_request = 0;
	goal_bearing = 0;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	GPIO_InitTypeDef GPIO_usrt;

	GPIO_usrt.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_usrt.GPIO_Mode = GPIO_Mode_AF;
	GPIO_usrt.GPIO_OType = GPIO_OType_PP;
	GPIO_usrt.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_usrt.GPIO_Speed = GPIO_Speed_40MHz;

	GPIO_Init(GPIOA,&GPIO_usrt);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	USART_Cmd(USART2, ENABLE);

	//interrupt
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	//USART_ITConfig(USART2, USART_IT_TC, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	  /* Enable the USARTx Interrupt */
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void PutcUART2(char ch){
	USART_SendData(USART2, (uint8_t) ch);

}

void USART2_IRQHandler(void){

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		char znak = USART_ReceiveData(USART2);

		//ak v predoslom byte bola poziadavka na zmenu ziadaneho uhla,
		//tak ciel zmeni ak dalsi byte je v zadanom rozsahu
		if (change_goal_request){

			if (znak >= 0 && znak <= GOAL_RANGE)
			goal_bearing = znak;

			change_goal_request = 0;
		}

		switch (znak){
		case 0xAA:   // vypinanie/zapinanie pohybu
			if (running)
					running = 0;
				else running = 1;
			break;
		case 0xBB: //poziadavka na zmenu ciela, v dalsom byte sa precita, aky bude novy ciel
			change_goal_request = 1;
		}

  /*  }else if(USART_GetITStatus(USART2, USART_IT_TC) != RESET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_TC);

		if (retazec[counter] != '\0'){
			PutcUART2(retazec[counter]);
			counter++;
		}*/
    }
}

void sendValue(){

	PutcUART2(goal_bearing);

	//sleep
	for (int i = 0; i < 50000; i++);
}

