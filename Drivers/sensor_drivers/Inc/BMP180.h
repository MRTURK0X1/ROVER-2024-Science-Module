
//https://controllerstech.com/interface-bmp180-with-stm32/
#ifndef USER_DRIVERS_INC_BMP180_H_
#define USER_DRIVERS_INC_BMP180_H_

#include "stm32f3xx_hal.h"
#include <math.h>

void BMP180_Init(I2C_HandleTypeDef* I2C);

void read_calliberation_data (void);

uint16_t Get_UTemp (void);

float BMP180_GetTemp (void);

uint32_t Get_UPress ();

float BMP180_GetPress ();

#endif
