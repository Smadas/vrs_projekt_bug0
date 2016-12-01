/*
 * kontrolka.h
 *
 *  Created on: Dec 1, 2016
 *      Author: Adam Sojka
 */

#ifndef KONTROLKA_H_
#define KONTROLKA_H_

long long gTimeStamp;

int init_kontrolka(void);
void blik_kontrolka(void);
int init_cas_blikanie();
void (*gBaseTimerHandler)(long long timeStamp);
void TIM2_IRQHandler(void);


#endif /* KONTROLKA_H_ */
