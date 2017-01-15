/*
 * bug_0.h
 *
 *  Created on: 11. 12. 2016
 *      Author: Michal Dobis
 *
 *      Mail: miso.dobis@gmail.com
 *      	  xdobis@stuba.sk
 *
 *  description:
 *  Kniznica hlavneho algoritmu bug_0.
 *  Implementacia je velmi jednoducha - v main staci zavolat funkcie:
 *   init() - ktora inicializuje hardver. Volanie na zaciatku main
 *   run() - jeden cyklus algoritmu. Musi byt volany pravidelne vo while cykle
 *
 *   V hlavnom algoritme je mozne vidiet posielanie dat na UART,
 *   tato funkcia sa ale vykona len v pripade, ze je spusteny mod debug
 *
 *   Popis algoritmu je mozne najst aj na wiki: https://github.com/Smadas/vrs_projekt_bug0/wiki
 *
 *   Blizsi popis k funkciam najdete priamo vo funkciach
 *
 */

/* includes */
/*----------------------------------------*/
#include <bluetooth.h>
#include <motorctrl.h>
#include "HMC5883L.h"
#include "i2c.h"
#include "runIndicatorLED.h"
#include "sensor.h"
/*----------------------------------------*/

#ifndef BUG_0_H_
#define BUG_0_H_

/* define constants */
/*----------------------------------------*/
//minimalne vzdialenosti prekazok pre rozne pripady
#define MIN_FRONT_DISTANCE 10
#define MIN_SIDE_DISTANCE 25
#define MIN_SIDE_END_AVOIDANCE_DISTANCE 55
#define MIN_SIDE_CRASH_DISTANCE 10

#define BEARING_ACCURACY 20 //presnost kompasu
#define FORWARD_SPEED 10 	//rychlost pre funkciu go_forward()
/*----------------------------------------*/

/* define functions */
/*----------------------------------------*/
#define abs(x)  (x<0)?-x:x
/*----------------------------------------*/

/* functions declaration */
/*----------------------------------------*/
int init();							//inicializacia, musi sa volat na zaciatku main
void run();							//jedna cyklus program, musi sa volat pravidelne v cykle
void stop();						//zastavenie vozidla
void go_forward();					//vozidla sa rozbehne rovno definovanou rychlostou FORWARD_SPEED
void turn_left(int speed);			//otocanie vlavo na mieste
void turn_left_one_wheel(int speed);//otacanie vlavo jednym kolesom
void turn_right(int speed);			//otacanie vpravo na mieste
void turn_right_one_wheel(int speed);//otacanie vpravo jednym kolesom
int turn(int angle_new, int speed);	 //otacanie na ziadany smer podla kompasu
									 // !!! POZOR !!! vo funkcii je while cyklus,
									//ktory skonci v pripade, ze sa robot otoceni na ziadany
									//smer, alebo zachyty prekazku pocas otacania
void initSendBearingTimer();	//casovac pravidelneho odosielania smerovania na seriovu linku
/*----------------------------------------*/


/* global variables */
/*----------------------------------------*/
//vzdialenosti od prekazok, data zo zvukacov
double obstacle_forward;
double obstacle_right;
double obstacle_left;

//uhol z kompasu
int bearing;

//flag, ktora detekuje, ci je momentalne aktivne obchadzanie prekazok
int avoidance_aktiv;
/*----------------------------------------*/

#endif /* BUG_0_H_ */
