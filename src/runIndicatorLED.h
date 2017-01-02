/*
 * runIndicatorLED.h
 *
 *  Created on: Dec 1, 2016
 *      Author: Adam Sojka
 *
 *  Kniznica, ktora pomocou casovaca TIM6 spusta LED pripojenu na port A pin PA5.
 *  Tato LED sa kazdu sekundu vypne alebo zapne.
 */

#ifndef RUNINDICATORLED_H_
#define RUNINDICATORLED_H_

void init_indicator_LED(void);
void init_indicator_LED_pin(void);
void init_indicator_LED_trigtim(void);
void init_indicator_LED_trigtim_interrupt(void);

#endif /* RUNINDICATORLED_H_ */
