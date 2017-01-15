/*
 * sensor.c
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

//Includes
#include <stddef.h>
#include <string.h>
#include "stm32l1xx.h"
#include "sensor.h"


//variables
void (*sensorCaptureHandler)(void);//smernik na funkciu ktora spracuje prerusenie CAPTURE_TIM

struct SensorCaptureStruc
{
	volatile uint32_t captureStep;//krokovanie nabezna/dobezna hrana
	volatile uint32_t risingTimeCapturing;
	volatile uint32_t risingTime;
	volatile uint32_t fallingTime;
}LeftSensorCaptureStruc, RightSensorCaptureStruc, ForwardSensorCaptureStruc;

volatile int selectSensor = 0;

//defines
	//cpu
#define STM_SYSTEM_CLOCK 16000000

	//vypocet vzdialenosti konstanty
#define DISTANCE_ENV_CONST 58.0
#define DISTANCE_CLC_CONST 0.1
#define DISTANCE_MAX 400

	//trig
#define LEFT_TRIG_PIN GPIO_Pin_9
#define RIGHT_TRIG_PIN GPIO_Pin_12
#define FORWARD_TRIG_PIN GPIO_Pin_11
#define TRIG_PORT GPIOA
#define TRIG_TIM_FREQ 100000
#define TRIG_TIM TIM7

	//capture
#define LEFT_TIM_CHANNEL_NUM 3
#define RIGHT_TIM_CHANNEL_NUM 1
#define FORWARD_TIM_CHANNEL_NUM 4
#define LEFT_TIM_CHANNEL TIM_Channel_3
#define RIGHT_TIM_CHANNEL TIM_Channel_1
#define FORWARD_TIM_CHANNEL TIM_Channel_4
#define LEFT_TIM_CC TIM_IT_CC3
#define RIGHT_TIM_CC TIM_IT_CC1
#define FORWARD_TIM_CC TIM_IT_CC4
#define LEFT_CAP_PIN GPIO_Pin_0
#define RIGHT_CAP_PIN GPIO_Pin_4
#define FORWARD_CAP_PIN GPIO_Pin_1
#define CAPTURE_TIM TIM3
#define CAPTURE_PORT GPIOB
#define LEFT_CAP_PINSOURCE GPIO_PinSource0
#define RIGHT_CAP_PINSOURCE GPIO_PinSource4
#define FORWARD_CAP_PINSOURCE GPIO_PinSource1
#define CAPTURE_COUNT_MAX 0xFFFF
#define LEFT_TIM_GETCAPTURE TIM_GetCapture3(CAPTURE_TIM)
#define RIGHT_TIM_GETCAPTURE TIM_GetCapture1(CAPTURE_TIM)
#define FORWARD_TIM_GETCAPTURE TIM_GetCapture4(CAPTURE_TIM)
#define CAPTURE_CLC_PRESCALER (unsigned short)159

	//volanie senzorov
#define SENSOR_CALL_TIM TIM10
#define SENSOR_CALL_TIM_PERIOD 65
#define SENSOR_CALL_TIM_PRESCALE (unsigned short) (16000000 / 1000) - 1
#define SENSOR_CALL_TIM_CLOCKDIVISION 0
#define SENSOR_CALL_TIM_PREEMPTPRIORITY 3
#define SENSOR_CALL_TIM_SUBPRIORITY 3

//Functions
//inicializacia senzorov vzdialenosti
void sensorInit(void)
{
	initSensorCaptureStruc();
	sensorInitTriggerTimer();
	sensorInitTriggerPin();
	sensorInitCapturePins();
	sensorInitCaptureTimer();
	sensorInitCallTimer();
}
//inicializacia struktur merania vzdialenosti
void initSensorCaptureStruc(void)
{
	LeftSensorCaptureStruc.captureStep = 0;
	LeftSensorCaptureStruc.risingTimeCapturing = 0;
	LeftSensorCaptureStruc.risingTime = 0;
	LeftSensorCaptureStruc.fallingTime = 0;

	RightSensorCaptureStruc.captureStep = 0;
	RightSensorCaptureStruc.risingTimeCapturing = 0;
	RightSensorCaptureStruc.risingTime = 0;
	RightSensorCaptureStruc.fallingTime = 0;

	ForwardSensorCaptureStruc.captureStep = 0;
	ForwardSensorCaptureStruc.risingTimeCapturing = 0;
	ForwardSensorCaptureStruc.risingTime = 0;
	ForwardSensorCaptureStruc.fallingTime = 0;

}
//inicializacia casovaca pravidelne volajuceho meranie vzdialenosti
void sensorInitCallTimer(void)
{
	selectSensor = 0;
	//Structure for timer settings
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	//  clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);

	//CALL_TIM init prerusenie
	sensorInitCallTimerInterrupt();

	TIM_TimeBaseStructure.TIM_Period = SENSOR_CALL_TIM_PERIOD;
	TIM_TimeBaseStructure.TIM_ClockDivision = SENSOR_CALL_TIM_CLOCKDIVISION;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Prescaler = SENSOR_CALL_TIM_PRESCALE;
	TIM_TimeBaseInit(SENSOR_CALL_TIM, &TIM_TimeBaseStructure);
	// TIM Interrupts enable
	TIM_ITConfig(SENSOR_CALL_TIM, TIM_IT_Update, ENABLE);
	// enable counter
	TIM_Cmd(SENSOR_CALL_TIM, ENABLE);
}
//inicializacia preruseni casovaca volajuceho meranie dialkomermy
void sensorInitCallTimerInterrupt(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = SENSOR_CALL_TIM_PREEMPTPRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = SENSOR_CALL_TIM_SUBPRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//inicializacia casovaca, ktory generuje spustaci impulz
void sensorInitTriggerTimer(void)
{
	//vypocet delicky pre periodu 10us
	unsigned short prescalerValue = (unsigned short) (STM_SYSTEM_CLOCK / TRIG_TIM_FREQ) - 1;

	//struktura pre zakladny casovac TRIG_TIM
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	//spustenie hodinovych impulzov pre TRIG_TIM
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	//TRIG_TIM init prerusenie
	sensorInitTriggerTimerInterrup();

	//init struktura TRIG_TIM
	TIM_TimeBaseStructure.TIM_Period = 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
	TIM_TimeBaseInit(TRIG_TIM, &TIM_TimeBaseStructure);//zapisanie struktury

	//povolenie preruseni TRIG_TIM
	TIM_ITConfig(TRIG_TIM, TIM_IT_Update, ENABLE);
}
//inicializacia preruseni casovaca, ktory generuje spustaci impulz
void sensorInitTriggerTimerInterrup(void)
{
	//TRIG_TIM struct prerusenie
	NVIC_InitTypeDef NVIC_InitStructure;

	//init struktura prerusenie
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);//zapisanie struktury
}
//inicializacia pinov pre spustanie dialkomerov
void sensorInitTriggerPin(void)
{
	//spusti hodiny pre port A
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	//vytvorenie struktury GPIO
	GPIO_InitTypeDef gpioInitStruc;
	gpioInitStruc.GPIO_Mode = GPIO_Mode_OUT;
	gpioInitStruc.GPIO_OType = GPIO_OType_PP;
	gpioInitStruc.GPIO_Speed = GPIO_Speed_400KHz;

	//zapisanie inicializacnej struktury - left
	gpioInitStruc.GPIO_Pin = LEFT_TRIG_PIN;
	GPIO_Init(TRIG_PORT, &gpioInitStruc);//errror - remote failure E31

	//zapisanie inicializacnej struktury - right
	gpioInitStruc.GPIO_Pin = RIGHT_TRIG_PIN;
	GPIO_Init(TRIG_PORT, &gpioInitStruc);

	//zapisanie inicializacnej struktury - forward
	gpioInitStruc.GPIO_Pin = FORWARD_TRIG_PIN;
	GPIO_Init(TRIG_PORT, &gpioInitStruc);

}
//inicializacia casovaca pre meranie dlzky impulzu z dialkomera
void sensorInitCaptureTimer(void)
{
	//CAPTURE_TIM struct zaklad
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//CAPTURE_TIM struct capture
	TIM_ICInitTypeDef  TIM_ICInitStructure;

	//povolenie hodin pre CAPTURE_TIM
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	//init CAPTURE_TIM struct capture
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;

	//CAPTURE_TIM nastavenie kanalov
	TIM_ICInitStructure.TIM_Channel = LEFT_TIM_CHANNEL;//left
	TIM_ICInit(CAPTURE_TIM, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = RIGHT_TIM_CHANNEL;//right
	TIM_ICInit(CAPTURE_TIM, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = FORWARD_TIM_CHANNEL;//forward
	TIM_ICInit(CAPTURE_TIM, &TIM_ICInitStructure);

	//nastavenie pocitadla kvoli delicke hodinovych impuzov
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Prescaler = CAPTURE_CLC_PRESCALER;
	TIM_TimeBaseInit(CAPTURE_TIM, &TIM_TimeBaseStructure);

	//CAPTURE_TIM povolenie pocitadla
	TIM_Cmd(CAPTURE_TIM, ENABLE);

	//povolenie CC poziadavky na prerusenie
	TIM_ITConfig(CAPTURE_TIM, LEFT_TIM_CC, ENABLE);//left
	TIM_ITConfig(CAPTURE_TIM, RIGHT_TIM_CC, ENABLE);//right
	TIM_ITConfig(CAPTURE_TIM, FORWARD_TIM_CC, ENABLE);//forward

	//CAPTURE_TIM prerusenie init
	sensorInitCaptureTimerInterrup();
}

//inicializacia preruseni casovaca pre meranie dlzky impulzu z dialkomera
void sensorInitCaptureTimerInterrup(void)
{
	//CAPTURE_TIM struct prerusenie
	NVIC_InitTypeDef NVIC_InitStructure;

	//init TIM struct prerusenie
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
//inicializacia pinu pre meranie dlzky impulzu z dialkomera
void sensorInitCapturePins(void)
{
	//GPIO struct
	GPIO_InitTypeDef GPIO_InitStructure;

	//GPIOB povolenie hodin
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	//CAPTURE_TIM struct init
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

	//inicializacia pinu left
	GPIO_InitStructure.GPIO_Pin   = LEFT_CAP_PIN;
	GPIO_Init(CAPTURE_PORT, &GPIO_InitStructure);//zapisanie struktury
	GPIO_PinAFConfig(CAPTURE_PORT, LEFT_CAP_PINSOURCE, GPIO_AF_TIM3);
	//inicializacia pinu right
	GPIO_InitStructure.GPIO_Pin   = RIGHT_CAP_PIN;
	GPIO_Init(CAPTURE_PORT, &GPIO_InitStructure);//zapisanie struktury
	GPIO_PinAFConfig(CAPTURE_PORT, RIGHT_CAP_PINSOURCE, GPIO_AF_TIM3);
	//inicializacia pinu forward
	GPIO_InitStructure.GPIO_Pin   = FORWARD_CAP_PIN;
	GPIO_Init(CAPTURE_PORT, &GPIO_InitStructure);//zapisanie struktury
	GPIO_PinAFConfig(CAPTURE_PORT, FORWARD_CAP_PINSOURCE, GPIO_AF_TIM3);
}

//spustenie merania laveho dialkomeru
void leftSensorMeasure(void)
{
	//spustenie casovaca
	TIM_Cmd(TRIG_TIM, ENABLE);
	//spustenie trig impulzu
	GPIO_WriteBit(TRIG_PORT, LEFT_TRIG_PIN, Bit_SET);
	//nulovanie CaptureStep ak by nahodou meranie zblblo
	LeftSensorCaptureStruc.captureStep = 0;
	//pridelenie funkcie spracovanie prerusenia capture
	sensorCaptureHandler = leftSensorCaptureHandler;
}
//spustenie merania praveho dialkomeru
void rightSensorMeasure(void)
{
	//spustenie casovaca
	TIM_Cmd(TRIG_TIM, ENABLE);
	//spustenie trig impulzu
	GPIO_WriteBit(TRIG_PORT, RIGHT_TRIG_PIN, Bit_SET);
	//nulovanie CaptureStep ak by nahodou meranie zblblo
	RightSensorCaptureStruc.captureStep = 0;
	//pridelenie funkcie spracovanie prerusenia capture
	sensorCaptureHandler = rightSensorCaptureHandler;
}
//spustenie memrania predneho dialkomeru
void forwardSensorMeasure(void)
{
	//spustenie casovaca
	TIM_Cmd(TRIG_TIM, ENABLE);
	//spustenie trig impulzu
	GPIO_WriteBit(TRIG_PORT, FORWARD_TRIG_PIN, Bit_SET);
	//nulovanie CaptureStep ak by nahodou meranie zblblo
	ForwardSensorCaptureStruc.captureStep = 0;
	//pridelenie funkcie spracovanie prerusenia capture
	sensorCaptureHandler = forwardSensorCaptureHandler;
}

//handlery bude treba dat do suboru, ktory bude urcovat veci specificke pre dany hardver
//samotne funkcie ktore ma vykonat ale ostanu tu
//spracovanie prerusenia z TRIG_TIM, casovac pre spustaci impulz dialkomeru
void TIM7_IRQHandler(void)
{
	//tez do samostatnej funkcie alebo mozno bude stacit #define fjdkkdf nazov funkcie
	if (TIM_GetITStatus(TRIG_TIM, TIM_IT_Update) == SET)
	{
		TIM_Cmd(TRIG_TIM, DISABLE);
		//vypnutie impulzu na prislusnom spustacom pine
		if (GPIO_ReadOutputDataBit(TRIG_PORT, LEFT_TRIG_PIN) == 1)
		{
			GPIO_WriteBit(TRIG_PORT, LEFT_TRIG_PIN, Bit_RESET);//ukoncenie trig impulzu left
		}
		else if (GPIO_ReadOutputDataBit(TRIG_PORT, RIGHT_TRIG_PIN) == 1)
		{
			GPIO_WriteBit(TRIG_PORT, RIGHT_TRIG_PIN, Bit_RESET);//ukoncenie trig impulzu right
		}
		else if (GPIO_ReadOutputDataBit(TRIG_PORT, FORWARD_TRIG_PIN) == 1)
		{
			GPIO_WriteBit(TRIG_PORT, FORWARD_TRIG_PIN, Bit_RESET);//ukoncenie trig impulzu forward
		}
		TIM_ClearITPendingBit(TRIG_TIM, TIM_IT_Update);
	}
}
//spracovanie prerusenia z TIM3, casovac pre meranie dlzky impulzu z dialialkomerov
void TIM3_IRQHandler(void)
{
	//spustenie spracovania prerusenia CAPTURE_TIM pre prislusny senzor
	if (sensorCaptureHandler != 0)
	{
		sensorCaptureHandler();
	}
}
//spracovanie prerusenia z casovaca pre volanie merania vzdialenosti
void TIM10_IRQHandler(void)
{
	if (TIM_GetITStatus(SENSOR_CALL_TIM, TIM_IT_Update) == SET)
	{
		//volanie merania senzora, ktory je na rade
		switch (selectSensor)
		{
		case 0:
			leftSensorMeasure();
			break;
		case 1:
			forwardSensorMeasure();
			break;
		case 2:
			rightSensorMeasure();
			break;
		}
		//inkrementovanie volaneho senzora
		selectSensor++;
		if (selectSensor > 2)
		{
			selectSensor = 0;
		}
		TIM_ClearITPendingBit(SENSOR_CALL_TIM, TIM_IT_Update);
	}
}

//meranie dlzky impulzu lavy senzor
void leftSensorCaptureHandler(void)
{
	if (TIM_GetITStatus(CAPTURE_TIM, LEFT_TIM_CC) != RESET)
	{
		//zmaz CAPTURE_TIM priznak prerusenia
		TIM_ClearITPendingBit(CAPTURE_TIM, LEFT_TIM_CC);

		if(LeftSensorCaptureStruc.captureStep == 0)
		{
			//zachyt cas nabeznej hrany
			LeftSensorCaptureStruc.risingTimeCapturing = LEFT_TIM_GETCAPTURE;
			LeftSensorCaptureStruc.captureStep = 1;
		}
		else if(LeftSensorCaptureStruc.captureStep == 1)
		{
			//zachyt cas dobeznej hrany
			LeftSensorCaptureStruc.fallingTime = LEFT_TIM_GETCAPTURE;
			LeftSensorCaptureStruc.risingTime = LeftSensorCaptureStruc.risingTimeCapturing;
		}
	}
}
//meranie dlzky impulzu lavy senzor
void rightSensorCaptureHandler(void)
{
	if (TIM_GetITStatus(CAPTURE_TIM, RIGHT_TIM_CC) != RESET)
	{
		//zmaz CAPTURE_TIM priznak prerusenia
		TIM_ClearITPendingBit(CAPTURE_TIM, RIGHT_TIM_CC);

		if(RightSensorCaptureStruc.captureStep == 0)
		{
			//zachyt cas nabeznej hrany
			RightSensorCaptureStruc.risingTimeCapturing = RIGHT_TIM_GETCAPTURE;
			RightSensorCaptureStruc.captureStep = 1;
		}
		else if(RightSensorCaptureStruc.captureStep == 1)
		{
			//zachyt cas dobeznej hrany
			RightSensorCaptureStruc.fallingTime = RIGHT_TIM_GETCAPTURE;
			RightSensorCaptureStruc.risingTime = RightSensorCaptureStruc.risingTimeCapturing;
		}
	}
}
//meranie dlzky impulzu lavy senzor
void forwardSensorCaptureHandler(void)
{
	if (TIM_GetITStatus(CAPTURE_TIM, FORWARD_TIM_CC) != RESET)
	{
		//zmaz CAPTURE_TIM priznak prerusenia
		TIM_ClearITPendingBit(CAPTURE_TIM, FORWARD_TIM_CC);

		if(ForwardSensorCaptureStruc.captureStep == 0)
		{
			//zachyt cas nabeznej hrany
			ForwardSensorCaptureStruc.risingTimeCapturing = FORWARD_TIM_GETCAPTURE;
			ForwardSensorCaptureStruc.captureStep = 1;
		}
		else if(ForwardSensorCaptureStruc.captureStep == 1)
		{
			//zachyt cas dobeznej hrany
			ForwardSensorCaptureStruc.fallingTime = FORWARD_TIM_GETCAPTURE;
			ForwardSensorCaptureStruc.risingTime = ForwardSensorCaptureStruc.risingTimeCapturing;
		}
	}
}

//prevzatie nameranej vzdialenosti z laveho dialkomeru
double leftSensorGetDistance(void)
{
	uint32_t leftDistanceTime = 0;//konecna dlzka impulzu z dialkomeru
	double distance = 0;//konecna vzdialenost

	//vypocet trvania impulzu
	leftDistanceTime = computeEchoDuration(LeftSensorCaptureStruc.risingTime, LeftSensorCaptureStruc.fallingTime);

	//vypocet vzdialenosti z laveho senzoru
	distance = leftDistanceTime/DISTANCE_CLC_CONST/DISTANCE_ENV_CONST;
	if (distance > DISTANCE_MAX)
	{
		//senzor nic nezachytil
		return DISTANCE_MAX;
	}
	else
	{
		return distance;
	}
}
//prevzatie nameranej vzdialenosti z praveho dialkomeru
double rightSensorGetDistance(void)
{
	uint32_t rightDistanceTime = 0;//konecna dlzka impulzu z dialkomer
	double distance = 0;//konecna vzdialenost

	//vypocet trvania impulzu
	rightDistanceTime = computeEchoDuration(RightSensorCaptureStruc.risingTime, RightSensorCaptureStruc.fallingTime);

	//vypocet vzdialenosti z praveho senzoru
	distance = rightDistanceTime/DISTANCE_CLC_CONST/DISTANCE_ENV_CONST;
	if (distance > DISTANCE_MAX)
	{
		//senzor nic nezachytil
		return DISTANCE_MAX;
	}
	else
	{
		return distance;
	}
}
//prevzatie nameranej vzdialenosti z predneho dialkomeru
double forwardSensorGetDistance(void)
{
	uint32_t forwardDistanceTime = 0;//konecna dlzka impulzu z dialkomeru
	double distance = 0;//konecna vzdialenost

	//vypocet trvania impulzu
	forwardDistanceTime = computeEchoDuration(ForwardSensorCaptureStruc.risingTime, ForwardSensorCaptureStruc.fallingTime);

	//vypocet vzdialenosti z predneho senzoru
	distance = forwardDistanceTime/DISTANCE_CLC_CONST/DISTANCE_ENV_CONST;
	if (distance > DISTANCE_MAX)
	{
		//senzor nic nezachytil
		return DISTANCE_MAX;
	}
	else
	{
		return distance;
	}
}

//vypocet trvania impulzu ozveny z dialkomera
uint32_t computeEchoDuration(uint32_t risingTime, uint32_t fallingTime)
{
	uint32_t distanceTime = 0;
	//vypocet podla toho ci nabezna alebo dobezna je vacsie cislo
	if (fallingTime > risingTime)
	{
		distanceTime = (fallingTime - risingTime) - 1;
	}
	else if (fallingTime < risingTime)
	{
		distanceTime = ((CAPTURE_COUNT_MAX - risingTime) + fallingTime) - 1;
	}
	else
	{
		distanceTime = 0;
	}

	return distanceTime;
}

