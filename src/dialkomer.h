/*
 * dialkomer.h
 *
 *  Created on: Nov 30, 2016
 *      Author: Adam Sojka
 */

#ifndef DIALKOMER_H_
#define DIALKOMER_H_

volatile uint32_t TIM4Freq;
volatile uint32_t pamatCapture[100];
volatile uint32_t pocitadlo;

int meraj_dialkomer(int cislo_senzoru);
int init_dialkomery(void);
int init_spustac_dialkomer(void);
int init_cas_zachyt_imp_dialkomer(void);
int init_cas_trig_dialkomer(void);
double citaj_vzdialenost(int cislo_senzoru);


#endif /* DIALKOMER_H_ */
