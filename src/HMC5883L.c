/*
 * HMC5883L.c
 *
 *  Created on: Dec 18, 2016
 *      Author: pozdr
 */

#include "HMC5883L.h"
#include "i2c.h"


Status readDataHMC5883L(unsigned int*data, unsigned char registerAddress)
{
	unsigned char buffer;
	buffer = 255;

	//Status error = I2C_Master_BufferReadWithoutRegisterAddress(buffer, 1, HMC5883L_ADDRESS_R);
	Status error = readByteI2C1(HMC5883L_ADDRESS_R, registerAddress, &buffer);
	//readByteI2C1()
	*data = buffer;

	return error;
}

Status writeBytesToCompass(void)
{

	Status errStat1 = writeByteI2C1(HMC5883L_ADDRESS_W, 0x00, 0x70); // Configuration Register A
	Status errStat2 = writeByteI2C1(HMC5883L_ADDRESS_W, 0x01, 0x0A); // Configuration Register B
	Status errStat3 = writeByteI2C1(HMC5883L_ADDRESS_W, 0x02, 0x00);	// Mode Register - Set Continuous-measurement mode
	return errStat3;
//	if(errStat1 == 'Success' && errStat2 == 'Success' && errStat1 == 'Success'){
//		return 'Success';
//	}
//	else{
//		return 'Config error';
//	}

}
