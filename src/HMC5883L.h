/*
 * HMC5883L.h
 *
 *  Created on: Dec 18, 2016
 *      Author:
 */

#ifndef HMC5883L_H_
#define HMC5883L_H_

#include "i2c.h"
#include "stm32l1xx.h"

//adresy I2C magnetometer
#define HMC5883L_ADDRESS_W 0x3C
#define HMC5883L_ADDRESS_R 0x3D

Status readDataHMC5883L(unsigned int*data, unsigned char registerAddress);
Status inicializaciaKompas(void);
Status writeBytesToCompass(void);
void initTimerMagnetometer(void);
int compass_get_heading(void);

#endif /* HMC5883L_H_ */
