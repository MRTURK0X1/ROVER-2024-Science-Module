#include <MQ.h>

void MQ_Init(MQ*mq,ADC_HandleTypeDef*adc,uint8_t temp,uint8_t hum){

	mq->temp = temp;
	mq->hum = hum;
	mq->ADC = adc;
	mq->R0 = getR0(mq);

}

void update_tempHum(MQ*mq135,uint8_t temp,uint8_t hum){
	mq135->temp = temp;
	mq135->hum = hum;
}

float getVoltage(ADC_HandleTypeDef* hadc) { //an average function

	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, 100);
	uint16_t adcRead = HAL_ADC_GetValue(hadc);
	HAL_ADC_Stop(hadc);

	return  (float)adcRead * Vstep; // PRODUCT With v step

}

float getCorrectionFactor(MQ*mq135) {
    return (CorrA * pow(mq135->temp, 3) + CorrB * pow(mq135->temp, 2) - CorrC * mq135->temp + CorrD - (CorrE * mq135->hum - CorrE * 33));
}

float getResistance(MQ*mq135) {
    float voltage = getVoltage(mq135->ADC);
    float rs = ((5 * RL) / voltage) - (float)RL;
    if (rs < 0) {
        rs = 0;
    }
    return rs;
}

float getCorrectedResistance(MQ*mq135) {
    return getResistance(mq135) / getCorrectionFactor(mq135);
}


float getR0(MQ*mq135) {

	float sum;

	for(uint8_t i = 0;i<50;i++){
    sum = sum + getResistance(mq135);
    HAL_Delay(100);
	}
	float r0  = sum/50;
    return r0;
}

float getCorrectedR0(MQ*mq135) {
    float r0 = getCorrectedResistance(mq135);
    return r0;
}


float getPPM(MQ*mq,float m,float b) {
    float ratio = getResistance(mq) / mq->R0;
    float ppm_log = (log10(ratio) - b) / m;
    float ppm = pow(10, ppm_log);
    if (ppm < 0) {
        ppm = 0;
    }
    return ppm;
}


float getCorrectedPPM(MQ*mq,float m,float b) {
    float ratio = getCorrectedResistance(mq) / mq->R0;
    float ppm_log = (log10(ratio) - b) / m;
    float ppm = pow(10, ppm_log);
    if (ppm < 0) {
        ppm = 0;
    }
    return ppm;
}


float getCorrectedCO2(MQ*mq) {
    return getCorrectedPPM(mq,110.47, -2.862) + ATMOCO2;
}

float getCO2(MQ*mq) {
    return getPPM(mq,110.47, -2.862) + ATMOCO2;
}

float getCorrectedNH3(MQ*mq) {
    return getCorrectedPPM(mq,-0.2640,-0.2307);
}

float getNH3(MQ*mq) {
    return getPPM(mq,-0.2640,-0.2307);
}

float getCorrectedCH4(MQ*mq) {
    return getCorrectedPPM(mq,-0.600553,0.801659);
}

float getCH4(MQ*mq) {
    return getPPM(mq,-0.600553,0.801659);
}
