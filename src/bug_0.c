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
	avoidance_aktiv = 0;

	  initUSART3();
	  //inicializacia motorov
	  Motor_init();
	  //inicializacia i2c a kompasu
	initI2C1();
	//Status errStat1 =writeBytesToCompass();

	writeBytesToCompass();
	return 1;
}


void run(){

	obstacle_forward = forwardSensorGetDistance();
	obstacle_right = rightSensorGetDistance();
	obstacle_left = leftSensorGetDistance();



	if (obstacle_right < MIN_SIDE_CRASH_DISTANCE){
		 turn_left_one_wheel(15);
		 sendValue(5);
		 return;
	}

	if (obstacle_left < MIN_SIDE_CRASH_DISTANCE){
		 turn_right_one_wheel(15);
		 sendValue(5);
		 return;
	}


	if (avoidance_aktiv){

		 if (obstacle_forward < MIN_FRONT_DISTANCE){
			 turn_left(12);
			 sendValue(4);

		 }
		 else if (obstacle_right > MIN_SIDE_DISTANCE + 30){

			// stop();
			 avoidance_aktiv = 0;
			 sendValue(2);
			 }

		 else if (obstacle_right > MIN_SIDE_CRASH_DISTANCE && obstacle_left > MIN_SIDE_CRASH_DISTANCE){
		// else{
		 go_forward();
		 sendValue(3);
		 return;
		}
	} else {

		 if (obstacle_forward > MIN_FRONT_DISTANCE){
			int ret = turn(goal_bearing, 7); //pokus o vratenie sa na ziadany smer


			 if (ret > 0){
				go_forward();
				sendValue(0);
			 }
			else if (ret == 0){     // pri otacani zdetekoval prekazku, cize musi prejst
				avoidance_aktiv = 1; //cize musi prejst do modu avoidance
				sendValue(1);
			}else{

				return;
			}
		 }
		 else{
			 stop();
			 avoidance_aktiv = 1;
		 }
	}


		/*if (bearing_error < 0 && obstacle_left > MIN_SIDE_DISTANCE){

			bearing = readDataCompass();
			turn(bearing + 90, 7); //otocenie vlavo o 90 stupnov
			bearing_error++;

		} else if (bearing_error > 0 && obstacle_right > MIN_SIDE_DISTANCE){

			bearing = readDataCompass();
			turn(bearing - 90, 7); //otocenie vpravo o 90 stupnov
			bearing_error--;

		} else if (bearing_error == 0){
			//ak by bol robot vychyleny od ziadeneho uhlu, tak sa dostane naspat na ziadany uhol
			turn(goal_bearing, 7);
		}

		if (obstacle_forward > MIN_FRONT_DISTANCE){
			go_forward();
		} else {
			bearing = readDataCompass();
			turn(bearing + 90, 7); //otocenie vlavo o 90 stupnov
			bearing_error++;
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

void turn_left(int speed){
	left_motor_set_speed(-speed + 1);
	right_motor_set_speed(speed);
}

void turn_left_one_wheel(int speed){
	right_motor_set_speed(speed);
	left_motor_set_speed(0);

}

void turn_right(int speed){
	left_motor_set_speed(speed + 1);
	right_motor_set_speed(-speed);
}

void turn_right_one_wheel(int speed){
	left_motor_set_speed(speed + 1);
	right_motor_set_speed(0);
}


int turn(int request_angle, int speed){

	//bearing = readDataCompass();
	bearing = compass_get_heading();

	if (request_angle > 360)
		request_angle -= 360;


	int angle_diff = bearing - request_angle;

	int error;
	if (angle_diff > BEARING_ACCURACY || angle_diff < -BEARING_ACCURACY)
		error = 1;
	else {
		stop();
		return 1;
		error = 0;
	}

	while (error){

		if (!running){ //v pripade, ze pocas otacania pride poziadavka na zastavenie,
			stop();     //tak treba ist von z cyklu
			return -1;
		}
		bearing = compass_get_heading();
		angle_diff = bearing - request_angle;

	//	if (angle_diff < 0)
		//	PutcUART3('-');

		sendValue(abs((char)angle_diff));

		if (angle_diff > BEARING_ACCURACY || angle_diff < -BEARING_ACCURACY){
			error = 1;
		}
		else{
			error = 0;
			return 1;
		}

		//obstacle_forward = forwardSensorGetDistance();
		obstacle_right = rightSensorGetDistance();
		// if (obstacle_forward < MIN_FRONT_DISTANCE + 10){
		if (obstacle_right < MIN_SIDE_DISTANCE){
			stop();
			return 0;

		 }
			if (angle_diff > 180)
				turn_left(speed);
			else if (angle_diff <= 180 && angle_diff > 0)
				turn_right(speed);
			else if (angle_diff <= 0 && angle_diff > - 180)
				turn_left(speed);
			else turn_right(speed);
		}

	return 1;
}

