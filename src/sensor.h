/*
 * sensor.h
 *
 *  Created on: Dec 3, 2016
 *      Author: Adam Sojka
 *
 * Description:
 * Tato kniznica obsahuje funkcie pre inicializaciu a obsluhu troch
 * ultrazvukovych dialkomerov so spustacim impulzom a impulzom ozveny.
 * Po inicializacii sa dialkomery spustaju pravidelne casovacom kazdych
 * 65ms jeden. Prislusnymi funkciami sa uz len prebera namerana hodnota.
 *
 * Dialkomery sa inicializuju spustenim funkcie sensorInit()
 * Namerane hodnoty sa preberaju funkciami: leftSensorGetDistance(),
 * rightSensorGetDistance(), forwardSensorGetDistance().
 *
 * Peripherals:
 * PB0, PB1, PB4 - capture
 * PA9, PA11, PA12 - trigger
 *
 * TIM3 - capture
 * TIM7 - trigger
 * TIM10 - calling
 *
 */

#ifndef SENSOR_H_
#define SENSOR_H_

//inicializacia senzorov vzdialenosti
void sensorInit(void);

//inicializacia struktur merania vzdialenosti
void initSensorCaptureStruc(void);
//inicializacia casovaca pravidelne volajuceho meranie vzdialenosti
void sensorInitCallTimer(void);
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
//inicializacia preruseni casovaca volajuceho meranie dialkomermy
void sensorInitCallTimerInterrupt(void);
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
uint32_t computeEchoDuration(uint32_t risingTime, uint32_t fallingTime);

#endif /* SENSOR_H_ */
