#ifndef MQ_H
#define MQ_H

#include "stm32f3xx_hal.h"
#include <math.h>
/// Resistor on Sensor in kΩ
#define RL 10

/// Voltage on Sensor in V
#define VIn 3.3

/// Board analog Input Resolution
/// Default: 2^12
//#define Resolution 4095

/// CO2 Level in Atmosphere
#define ATMOCO2 397.13

//Correction Values
#define CorrA -0.000002469136
#define CorrB 0.00048148148
#define CorrC 0.0274074074
#define CorrD 1.37530864197
#define CorrE 0.0019230769

#define Vstep (float)VIn/(4096-1)



typedef struct _MQ{

	ADC_HandleTypeDef* ADC;
	float R0;
	uint8_t temp;
	uint8_t hum;

}MQ;

void MQ_Init(MQ*mq,ADC_HandleTypeDef*adc,uint8_t temp,uint8_t hum);

void update_tempHum(MQ*mq135,uint8_t temp,uint8_t hum);

/// Assume CO2 Level is the default Atmospheric Level (~400ppm)
float getR0(MQ*mq);
float getCorrectedR0(MQ*mq);

/// Gets the resolved sensor voltage
float getVoltage(ADC_HandleTypeDef* hadc);

/// Calculate Correction Factor depending on temparature and humidity
float getCorrectionFactor(MQ*mq);

/// Calculates the Resistance of the Sensor
float getResistance(MQ*mq);
float getCorrectedResistance(MQ*mq);
/// Calculates ppm on a exponential curve
/// (Different Gases have different curves)
float getPPM(MQ*mq, float m, float b);
float getCorrectedPPM(MQ*mq, float m, float b);

/// Gets ppm of ammonia in Air (NH3)
float getNH3(MQ*mq);
float getCorrectedNH3(MQ*mq);
float getNH3_a(MQ*mq);
/// Gets ppm of CO2 in Air
float getCO2(MQ*mq);
float getCorrectedCO2(MQ*mq);

float getCH4(MQ*mq);
float getCorrectedCH4(MQ*mq);

#endif

