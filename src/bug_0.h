/*
 * bug_0.h
 *
 *  Created on: 11. 12. 2016
 *      Author: michal1
 */

#include <bluetooth.h>
#include <motorctrl.h>

#ifndef BUG_0_H_
#define BUG_0_H_

//minimalna vzdialenost prekazky
#define MIN_DISTANCE 30

int init();
void run();
void stop();
void go_forward();
void turn(double angle_change);
void calibrate_movement();


//vzdialenosti od prekazok, data zo zvukacov
double obstacle_forward;
double obstacleRight;
double obstacleLeft;

//uhol z kompasu
double bearing;

//pomocna premenna, ktora detekuje kolko krat sa robot musel otacat
int bearing_error;

#endif /* BUG_0_H_ */
