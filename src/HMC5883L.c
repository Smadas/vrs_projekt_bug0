/*
 * HMC5883L.c
 *
 *  Created on: Dec 18, 2016
 *      Author: pozdr
 */

#include "HMC5883L.h"
#include "i2c.h"

//global variables
volatile int compassHeading = 0;

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

	initTimerMagnetometer();

	return errStat3;
//	if(errStat1 == 'Success' && errStat2 == 'Success' && errStat1 == 'Success'){
//		return 'Success';
//	}
//	else{
//		return 'Config error';
//	}

}

//casovac pravidelne spustajuci meranie magnetometra
void initTimerMagnetometer(void)
{
	unsigned short prescalerValue = (unsigned short) (16000000/1000) - 1;//TIM_CLC_PRESCALER;

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = 50 - 1;//TIM_PERIOD;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;//TIM_CLC_DIV;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
	TIM_TimeBaseInit(TIM11, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM11, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM11, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = TIM11_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//TIM_INTERRUPT_PREEMP;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//TIM_INTERRUPT_SUB;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//spracovanie prerusenia z TIM6, casovaca pre kontrolku
void TIM11_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM11, TIM_IT_Update) == SET)
	{
		compassHeading = readDataCompass();
		TIM_ClearITPendingBit(TIM11, TIM_IT_Update);
	}
}

//vycitanie hodnoty smerovania
int compass_get_heading(void)
{
	return compassHeading;
}
