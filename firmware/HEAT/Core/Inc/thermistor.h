#ifndef THERMISTOR_H
#define THERMISTOR_H

#include "stm32f1xx_hal.h"

void    Thermistor_Init(ADC_HandleTypeDef *hadc);
float   Thermistor_ReadTemperatureC(void);


#endif
