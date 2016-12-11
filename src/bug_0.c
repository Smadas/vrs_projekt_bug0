/*
 * bug_0.c
 *
 *  Created on: 11. 12. 2016
 *      Author: michal1
 */

#include <bug_0.h>


int init(){

	//inicializacia premennych
	obstacle_forward = 0;
	obstacleRight = 0;
	obstacleLeft = 0;
	bearing = 0;
	bearing_error = 0;

	  initUSART2();
	//inicializacia zvukacov, kompasu a motorov
	return 1;
}

void run(){

	//bluetooth.start();
	while (1){

		//bluetooth.getGoal();
		//sensor.getDistance();
		if (bearing_error > 0 && obstacleLeft > MIN_DISTANCE){

			turn(-90);

		} else if (bearing_error < 0 && obstacleRight > MIN_DISTANCE){

			turn(90);

		} else{
			//ak by bol robot vychyleny od ziadeneho uhlu, tak sa dostane naspat na ziadany uhol
			calibrate_movement();
		}

		if (obstacle_forward > MIN_DISTANCE){
			go_forward();
		} else {
			turn(90);
		}
	}
}

void stop(){

	//leftMotor.setSpeed(0);
	//rightMotor.setSpeed(0);
}

void go_forward(){

	//leftMotor.setSpeed(x);
	//rightMotor.setSpeed(x);

}

void turn(double angle_change){

	double request_angle = bearing + angle_change;

	if (request_angle > 360)
		request_angle -= 360;

	int angle_diff = request_angle - bearing; //treba vyriesit problem, ked sa bude otacat cez sever, 360-0.

	while (angle_diff > 1 || angle_diff < -1){ //1 a -1 stupen znamena pozadovanu presnost otacania
		//leftMotor.setSpeed(x);
		//rightMotor.setSpeed(-x);
		angle_diff = request_angle - bearing;
	}

	if (angle_change > 0){
		bearing_error++;
	}
	else bearing_error--;

}

void calibrate_movement(){

	int angle_diff = goal_bearing - bearing; //treba vyriesit problem, ked sa bude otacat cez sever, 360-0.

	while (angle_diff > 1 || angle_diff < -1){ //1 a -1 stupen znamena pozadovanu presnost otacania
		//leftMotor.setSpeed(x);
		//rightMotor.setSpeed(-x);
		angle_diff = goal_bearing - bearing;
	}

	bearing_error = 0;

}

