/*
 * serial.h
 *
 *  Created on: Dec 11, 2016
 *      Author: Adam Sojka
 *
 *      Tato kniznica sluzi na spustenie overenie funkcnosti dialkomerov. Namerane vzdialenosti
 *      sa odosielaju na USART.
 */


#ifndef SERIAL_H_
#define SERIAL_H_

uint16_t receivedChar;
volatile int USARTbufferRDY;
volatile char USARTbufferOut[100]; //buffer, kde su ulozene znaky na odoslanie
volatile int USARTbufferInkr;

void inicializaciaPrerusenieUSART(void);
void inicializaciaUSART(void);
int rychlostBlikaniaLED(int blikacRychlost,uint16_t value);
void PutcUART2(char *ch);
void USART2_IRQHandler(void);
int odoslanieRetazca(int tvarVypisu);

//dialkomer - vytvorenie spravy so vzdialenostou
int sensorMessage(double distance, int sensorNum);
int sensorMessageAll(double leftDistance, double rightDistance, double forwardDistance);

#endif /* SERIAL_H_ */
