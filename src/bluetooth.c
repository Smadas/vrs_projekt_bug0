/*
 * bluetooth.c
 *
 *  Created on: 11. 12. 2016
 *      Author: michal1
 */


#include <bluetooth.h>

void initUSART3(){

	running = 0;
	change_goal_request = 0;
	goal_bearing = 0;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);


	GPIO_InitTypeDef GPIO_usrt;

	GPIO_usrt.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_usrt.GPIO_Mode = GPIO_Mode_AF;
	GPIO_usrt.GPIO_OType = GPIO_OType_PP;
	GPIO_usrt.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_usrt.GPIO_Speed = GPIO_Speed_40MHz;

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);

	GPIO_Init(GPIOC, &GPIO_usrt);

//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	//GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 19200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
	USART_Cmd(USART3, ENABLE);

	//interrupt
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	//USART_ITConfig(USART2, USART_IT_TC, ENABLE);

	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	  /* Enable the USARTx Interrupt */
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//USART_Cmd(USART1, ENABLE);
}

void PutcUART3(char ch){
	USART_SendData(USART3, (uint8_t) ch);

}

void USART3_IRQHandler(void){

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		char znak = USART_ReceiveData(USART3);

		//ak v predoslom byte bola poziadavka na zmenu ziadaneho uhla,
		//tak ciel zmeni ak dalsi byte je v zadanom rozsahu
		if (change_goal_request){

			if (znak >= '0' && znak <= '7')
			goal_bearing = znak;

			change_goal_request = 0;
		}

		switch (znak){
		case 'x':   // vypinanie/zapinanie pohybu
			if (running)
					running = 0;
				else running = 1;
			break;
		case 'c': //poziadavka na zmenu ciela, v dalsom byte sa precita, aky bude novy ciel
			change_goal_request = 1;
			break;
		default:
			change_goal_request = 0;
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

void sendValue(double prekazka){
	//PutcUART3('m');



	if (running)
		PutcUART3((char)prekazka);
	else
		PutcUART3(0xFF);

	//sleep
	for (int i = 0; i < 250000; i++);
}

