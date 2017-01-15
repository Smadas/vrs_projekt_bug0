/*
 * runIndicatorLED.h
 *
 *  Created on: Dec 1, 2016
 *      Author: Adam Sojka
 *
 *	Description:
 *	Kniznica, ktora pomocou casovaca TIM6 spusta LED pripojenu na port A pin PA5.
 *	Tato LED sa kazdu sekundu vypne alebo zapne. Blikanie LED sluzi na indikaciu
 *	ci MCU pracuje spravne.
 *
 *	Peripherals:
 *	PA5 - vystup na LED
 *	TIM6 - spustaci casovac
 *
 */

#ifndef RUNINDICATORLED_H_
#define RUNINDICATORLED_H_

void init_indicator_LED(void);
void init_indicator_LED_pin(void);
void init_indicator_LED_trigtim(void);
void init_indicator_LED_trigtim_interrupt(void);

#endif /* RUNINDICATORLED_H_ */
