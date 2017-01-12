/*
 * bug_0.h
 *
 *  Created on: 11. 12. 2016
 *      Author: michal1
 */

#include <bluetooth.h>
#include <motorctrl.h>
#include "HMC5883L.h"
#include "i2c.h"

#ifndef BUG_0_H_
#define BUG_0_H_

//minimalna vzdialenost prekazky
#define MIN_FRONT_DISTANCE 10
#define MIN_SIDE_DISTANCE 25
#define MIN_SIDE_CRASH_DISTANCE 10

#define BEARING_ACCURACY 10

#define abs(x)  (x<0)?-x:x

int init();
void run();
void stop();
void go_forward();
void turn_left(int speed);
void turn_right(int speed);
int turn(int angle_new, int speed);

//vzdialenosti od prekazok, data zo zvukacov
double obstacle_forward;
double obstacle_right;
double obstacle_left;

//uhol z kompasu
int bearing;

int avoidance_aktiv;

//pomocna premenna, ktora detekuje kolko krat sa robot musel otacat
int bearing_error;

#endif /* BUG_0_H_ */
