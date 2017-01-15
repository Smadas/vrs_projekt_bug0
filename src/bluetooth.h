/*
 * bluetooth.h
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

#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

/* includes  */
/*----------------------------------------*/
#include <stddef.h>
#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
/*----------------------------------------*/


/* global variables */
/*----------------------------------------*/
volatile int running;				//indikuje, ci operator umoznil vykonavat algoritmus
int goal_bearing;			//ziadany smer od operatora
int debug;					//debug mode

int change_goal_request;	//pomocna premenna. Setuje sa na 1, ak pride znak 'c'.
							//Resetuje sa ked, pride dalsi znak
/*----------------------------------------*/

/* functions declarations */
/*----------------------------------------*/
void initUSART3();					//inicializacia
void PutcUART3(char ch);			//posielanie znaku
void USART3_IRQHandler(void);		//IRQ handler
void sendValue(double variable);	//ak bude v mode debug a v stave running tak funkcia bude posielat
									//na UART hodnotu vo variable. Ak v debug nebude, tak sa nebude posielat nic.
									//pokial nebude v stave running tak funkcia bude posielat 0xFF vzdy
/*----------------------------------------*/


#endif /* BLUETOOTH_H_ */
