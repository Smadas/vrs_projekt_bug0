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

//inicializacia senzorov vzdialenosti
void sensorInit(void);

//inicializacia casovaca, ktory generuje spustaci impulz
void sensorInitTriggerTimer(void);
//inicializacia pinov pre spustanie dialkomerov
void sensorInitTriggerPin(void);
//inicializacia casovaca pre meranie dlzky impulzu z dialkomera
void sensorInitCaptureTimer(void);
//inicializacia preruseni casovaca, ktory generuje spustaci impulz
void sensorInitTriggerTimerInterrup(void);
//inicializacia preruseni casovaca pre meranie dlzky impulzu z dialkomera
void sensorInitCaptureTimerInterrup(void);
//inicializacia pinu pre meranie dlzky impulzu z dialkomera
void sensorInitCapturePins(void);

//spustenie merania laveho dialkomeru
void leftSensorMeasure(void);
//spustenie merania praveho dialkomeru
void rightSensorMeasure(void);
//spustenie memrania predneho dialkomeru
void forwardSensorMeasure(void);

//meranie dlzky impulzu lavy senzor
void leftSensorCaptureHandler(void);
//meranie dlzky impulzu lavy senzor
void rightSensorCaptureHandler(void);
//meranie dlzky impulzu lavy senzor
void forwardSensorCaptureHandler(void);

//prevzatie nameranej vzdialenosti z laveho dialkomeru
double leftSensorGetDistance(void);
//prevzatie nameranej vzdialenosti z praveho dialkomeru
double rightSensorGetDistance(void);
//prevzatie nameranej vzdialenosti z predneho dialkomeru
double forwardSensorGetDistance(void);
//vypocet trvania impulzu ozveny z dialkomera
uint16_t computeEchoDuration(uint16_t risingTime, uint16_t fallingTime);

#endif /* SENSOR_H_ */
