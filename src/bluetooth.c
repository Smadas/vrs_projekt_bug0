/*
 * bluetooth.c
 *
 *  Created on: 11. 12. 2016
 *      Author: Michal Dobis
 *
 *      Mail: miso.dobis@gmail.com
 *      	  xdobis@stuba.sk
 *
 *  description:
 *  Kniznica pre komunikaciu cez bluetooth resp. USART
 *  Pre implementaciu je nutne zavolat len funkciu initUSART3()
 *
 *  Kniznica obsahuje komunikacny protokol:
 *
 *  receive (ASCII format):
 *  x - aktivuje/deaktivuje running mode - v main programe nutne citat hodnotu running
 *  c - po prijati tejto hodnoty sa bude ocakavat hodnota
 * 	 	z rozsahu 0-7, ktora bude zadavat ziadany smer
 * 	0-7 - ak bola prijata hodnota 'c' a nasledne cislo z rozsahu 0-7 nastavi sa
 * 		  premenna goal_bearing = X*45 stupnov, kde X je hodnota 0-7
 * d - aktivuje/deaktivuje debug mode. Blizsi popis pri funkcii sendValue()
 *
 * send (hex format)
 * 0xFF - robot nie je v pohybe, caka na operatora kym neposle hodnotu 'x'
 * goal_bearing - ak bude v pohybe, tak posiela pravidelne ziadany smer
 * ine hodnoty - ak bude v mode debug
 *
 *   Popis komunikacneho protokolu najdete aj na wiki: https://github.com/Smadas/vrs_projekt_bug0/wiki
 *
 *   Blizsi popis k funkciam najdete priamo vo funkciach
 *
 */

#include <bluetooth.h>

void initUSART3(){

	//global variable init
	running = 0;
	change_goal_request = 0;
	goal_bearing = 0;
	debug = 0;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* GPIO INIT */
	GPIO_InitTypeDef GPIO_usrt;

	GPIO_usrt.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_usrt.GPIO_Mode = GPIO_Mode_AF;
	GPIO_usrt.GPIO_OType = GPIO_OType_PP;
	GPIO_usrt.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_usrt.GPIO_Speed = GPIO_Speed_40MHz;

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);

	GPIO_Init(GPIOC, &GPIO_usrt);

	/* USART INIT */
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

	  /* Enable the USARTx Interrupt */
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

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
		//tak ciel sa zmeni, ak dalsi byte je v zadanom rozsahu
		if (change_goal_request){

			if (znak >= '0' && znak <= '7')
			goal_bearing = (znak - 48) * 45; //ASCII 0 = 48
											//rozsah ziadanych hodnot 0-7, cize 360/8 = 45 stupnov
			change_goal_request = 0;
		}

		switch (znak){
		case 'x':   // vypinanie/zapinanie pohybu
			if (running) running = 0;
			else running = 1;
			break;
		case 'c': //poziadavka na zmenu ciela, v dalsom byte sa precita, aky bude novy ciel
			change_goal_request = 1;
			break;
		case 'd' :
			if (debug) debug = 0; // vypinanie/zapinanie modu debug
			else debug = 1;
		default:
			change_goal_request = 0;
		}
    }
}

void sendValue(double variable){

	if (running){
		if (debug){
		PutcUART3((char)variable);
		} else return;				//ak nie je debug, tak sa funkcia nema vykonavat

	}else PutcUART3(0xFF);			//pokial vozidlo nie je v stave running, tak to bude indikovat posielanim 0xFF
									//toto posielanie treba implementovat v main programe

	//sleep
	for (int i = 0; i < 250000; i++);
}



