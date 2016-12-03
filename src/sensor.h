/*
 * sensor.h
 *
 *  Created on: Dec 3, 2016
 *      Author: Adam Sojka
 *
 * Tato kniznica obsahuje funkcie pre inicializaciu a obsluhu troch
 * ultrazvukovych dialkomerov so spustacim impulzom a impulzom ozveny.
 */

#ifndef SENSOR_H_
#define SENSOR_H_

//global variables TEMP
volatile uint32_t TIM4Freq;
volatile uint32_t pamatCapture[100];
volatile uint32_t pocitadlo;


//inicializacia senzorov vzdialenosti
void sensorInit(void);

//inicializacia casovaca, ktory generuje spustaci impulz
void sensorInitTriggerTimer(void);
//inicializacia pinov pre spustanie dialkomerov
void sensorInitTriggerPin(void);
//inicializacia casovaca pre meranie dlzky impulzu z dialkomera
void sensorInitCaptureTimer(void);
//inicializacia pinu pre meranie dlzky impulzu z dialkomera
void sensorInitCapturePins(void);

//spustenie merania laveho dialkomeru
void leftSensorMeasure(void);
//spustenie merania praveho dialkomeru
void rightSensorMeasure(void);
//spustenie memrania predneho dialkomeru
void forwardSensorMeasure(void);

//prevzatie nameranej vzdialenosti z laveho dialkomeru
double leftSensorGetDistance(void);
//prevzatie nameranej vzdialenosti z praveho dialkomeru
double rightSensorGetDistance(void);
//prevzatie nameranej vzdialenosti z predneho dialkomeru
double forwardSensorGetDistance(void);

#endif /* SENSOR_H_ */
