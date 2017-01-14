/*
 * bug_0.c
 *
 *      Created on: 11. 12. 2016
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

#include <bug_0.h>
#include <sensor.h>

int init(){

	//inicializacia premennych
	obstacle_forward = 0;
	obstacle_right = 0;
	obstacle_left = 0;
	bearing = 0;
	avoidance_aktiv = 0;

	//inicializacia USART
	initUSART3();

	//inicializacia motorov
	Motor_init();

	//inicializacia i2c a kompasu
	initI2C1();
	writeBytesToCompass();

	//inicializacia LEDky
	init_indicator_LED();

	//inicializacia dialkomerov
	sensorInit();
	return 1;
}

//hlavny algoritmus
void run(){

	//prevzatie dat z dialkomerov
	obstacle_forward = forwardSensorGetDistance();
	obstacle_right = rightSensorGetDistance();
	obstacle_left = leftSensorGetDistance();

	//vozidlo je blizko pri prekazke na pravo, preto musi zacat zatacat do lava
	if (obstacle_right < MIN_SIDE_CRASH_DISTANCE){
		 turn_left_one_wheel(15);
		 sendValue(5);
		 return;
	}
	//vozidlo je blizko pri prekazke na lavo, preto musi zacat zatacat do prava
	if (obstacle_left < MIN_SIDE_CRASH_DISTANCE){
		 turn_right_one_wheel(15);
		 sendValue(5);
		 return;
	}

	//ak je v mode avoidance(obchadza prekazku)
	if (avoidance_aktiv){

		//pokial ma pred sebou prekazku, zacne sa jej vyhybat vlavo
		 if (obstacle_forward < MIN_FRONT_DISTANCE){
			 turn_left(12);
			 sendValue(4);
		 //ak na pravo sensore sa vzdiali od prekazky na urcitu vzdialenost,
		//zrusi sa mod avoidance a vozidlo sa tak moze pokusit opat sa dostat na ziadany smer
		 }else if (obstacle_right > MIN_SIDE_END_AVOIDANCE_DISTANCE){
			 avoidance_aktiv = 0;
			 sendValue(2);

		//pokial robot nie je v kolizii z bocnymi prekazkami, moze pokracovat rovno
		 }else if (obstacle_right > MIN_SIDE_CRASH_DISTANCE && obstacle_left > MIN_SIDE_CRASH_DISTANCE){
			go_forward();
			sendValue(3);
			return;
		 }
	} else {

		//pokial vozidlo pred sebou nema ziadnu prekazku, moze sa otocit
		//na svoj smer a ak bude uspesny tak ist rovno
		 if (obstacle_forward > MIN_FRONT_DISTANCE){

			 int ret = turn(goal_bearing, 7); //pokus o vratenie sa na ziadany smer

			 //ak je navratova hodnota kladna,
			 //tak sa robot dostal na ziadany smer a moze ist rovno
			 if (ret > 0){
				go_forward();
				sendValue(0);

			//ak je navratova hodnota 0, tak pri otacani zdetekoval prekazku,
			//cize musi prejst do modu avoidance
			 } else if (ret == 0){
				avoidance_aktiv = 1;
				sendValue(1);
			//zaporna hodnota sa vracia v pripade,
			//ze operator vydal prikaz na zastavenie vozidla pocas otacania sa
			}else{
				return;
			}
		 }
		 //robot ma pred sebou prekazku, musi zastavit a prepnut sa do modu avoidance
		 else{
			 stop();
			 avoidance_aktiv = 1;
		 }
	}
}

//zastavenie vozidla
void stop(){

	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

//vozidla ma ist rovno
void go_forward(){
	left_motor_set_speed(FORWARD_SPEED);
	right_motor_set_speed(FORWARD_SPEED);
}

//otacanie vlavo na mieste
void turn_left(int speed){
	left_motor_set_speed(-speed + 1);
	right_motor_set_speed(speed);
}

//otacanie vlavo jednym kolesom
void turn_left_one_wheel(int speed){
	right_motor_set_speed(speed);
	left_motor_set_speed(0);

}

//otacanie vpravo na mieste
void turn_right(int speed){
	left_motor_set_speed(speed + 1);
	right_motor_set_speed(-speed);
}

//otacanie vpravo jednym kolesom
void turn_right_one_wheel(int speed){
	left_motor_set_speed(speed + 1);
	right_motor_set_speed(0);
}


//otacanie sa na ziadany smer a konkretnou rychlostou
int turn(int request_angle, int speed){

	//prevzatie nameraneho smeru z kompasu
	bearing = compass_get_heading();

	//v pripade, zeby ziadany uhol bol vacsi ako 360
	if (request_angle > 360)
		request_angle -= 360;

	int angle_diff = bearing - request_angle;

	//ak sa robot nachadza v tolerancii, tak sa nemusi otacat
	if (!(angle_diff > BEARING_ACCURACY || angle_diff < -BEARING_ACCURACY)){
		stop();
		return 1;
	}

	while (1){

		//v pripade, ze pocas otacania pride poziadavka na zastavenie
		if (!running){
			stop();
			return -1;
		}

		//namerany uhol treba cyklicky obnovovat
		bearing = compass_get_heading();
		angle_diff = bearing - request_angle;

		//DEBUG: posielanie rozdielu aktualneho a nameraneho smeru
		sendValue(abs((char)angle_diff));

		//robot sa dostal do ziadaneho smeru
		if (angle_diff < BEARING_ACCURACY && angle_diff > -BEARING_ACCURACY){
			return 1;
		}

		//aktualizuje sa hodnota z praveho dialkomeru
		obstacle_right = rightSensorGetDistance();
		//v pripade ak pri otacani z detekoval prekazku na kraji treba zastavit
		//a vratit nulu, aby sa robot mohol prepnut do avoidance modu
		if (obstacle_right < MIN_SIDE_DISTANCE){
			stop();
			return 0;
		 }

		//podmienky otacania sa
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

