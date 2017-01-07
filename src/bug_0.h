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
#define MIN_FRONT_DISTANCE 30
#define MIN_SIDE_DISTANCE 20

#define BEARING_ACCURACY 10

#define abs(x)  (x<0)?-x:x

int init();
void run();
void stop();
void go_forward();
void turn_left(int x);
void turn_right(int x);
void turn(int angle_new);
void calibrate_movement();


//vzdialenosti od prekazok, data zo zvukacov
double obstacle_forward;
double obstacle_right;
double obstacle_left;

//uhol z kompasu
int bearing;

//pomocna premenna, ktora detekuje kolko krat sa robot musel otacat
int bearing_error;

#endif /* BUG_0_H_ */
