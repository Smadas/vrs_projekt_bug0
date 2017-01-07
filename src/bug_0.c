/*
 * bug_0.c
 *
 *  Created on: 11. 12. 2016
 *      Author: michal1
 */

#include <bug_0.h>
#include <sensor.h>


int init(){

	//inicializacia premennych
	obstacle_forward = 0;
	obstacle_right = 0;
	obstacle_left = 0;
	bearing = 0;
	bearing_error = 0;

	  initUSART3();
	  //inicializacia motorov
	  Motor_init();
	  //inicializacia i2c a kompasu
	initI2C1();
	Status errStat1 =writeBytesToCompass();
	return 1;
}


void run(){

	bearing = readDataCompass();

	sendValue(bearing);

	//bluetooth.start();
	if (running){
		/*obstacle_forward = forwardSensorGetDistance();
		obstacle_right = rightSensorGetDistance();
		sendValue(obstacle_right);

		if (obstacle_right > MIN_SIDE_DISTANCE){
			stop();
		}

		else if (obstacle_forward > MIN_FRONT_DISTANCE){
			//turn();
			go_forward();
		}
		else stop();*/

		turn(160);
		turn(250);

	}
	else{
		stop();
		sendValue(0);
	}

	/*while (1){

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
	}*/
}

void stop(){

	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

void go_forward(){

	left_motor_set_speed(10);
	right_motor_set_speed(10);

}

void turn_left(int x){
	left_motor_set_speed(x);
	right_motor_set_speed(-x);
}

void turn_right(int x){
	left_motor_set_speed(-x);
	right_motor_set_speed(x);
}

void turn(int request_angle){

	bearing = readDataCompass();
	//int request_angle = bearing + angle_change;

/*	if (request_angle > 360)
		request_angle -= 360;
*/

	//int angle_diff = request_angle - bearing; //treba vyriesit problem, ked sa bude otacat cez sever, 360-0.

	int angle_diff = bearing - request_angle;
	//int angle_diff_2 =
	int error;

	if (angle_diff > BEARING_ACCURACY || angle_diff < -BEARING_ACCURACY)
		error = 1;
	else error = 0;

	while (error){

		if (angle_diff > 180)
			turn_right(4);
		else if (angle_diff <= 180 && angle_diff > 0)
			turn_left(4);
		else if (angle_diff <= 0 && angle_diff > - 180)
			turn_right(4);
		else turn_left(4);

		bearing = readDataCompass();
		//sendValue(bearing);
		angle_diff = bearing - request_angle;

		if (angle_diff > BEARING_ACCURACY || angle_diff < -BEARING_ACCURACY)
			error = 1;
		else error = 0;

		/*if (rotation > 0){
			left_motor_set_speed(-8);
			right_motor_set_speed(8);

			if (angle_diff > BEARING_ACCURACY) error = 1;
			else error = 0;
		} else {
			left_motor_set_speed(8);
			right_motor_set_speed(-8);

			if (angle_diff < -BEARING_ACCURACY) error = 1;
			else error = 0;
		}*/

	}

	stop();
	/*if (angle_change > 0){
		bearing_error++;
	}
	else bearing_error--;*/

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

