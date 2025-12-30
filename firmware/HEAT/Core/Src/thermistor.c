/*
 * thermistor.c
 *
 * Created on: Nov 29, 2025
 * Author: dyord
 */


#include "thermistor.h"
#include <math.h>

// ---- NTC / ADC config ----
#define NTC_SUPPLY_V      3.3f
#define NTC_SERIES_R      100000.0f   // 100k series resistor
#define NTC_R0            100000.0f   // 100k at 25°C
#define NTC_T0_C          25.0f       // nominal temperature
#define NTC_BETA          3950.0f     // 3950 or 3960, close enough
#define ADC_MAX_VALUE     4095.0f

static ADC_HandleTypeDef *s_hadc = NULL;

void Thermistor_Init(ADC_HandleTypeDef *hadc)
{
    s_hadc = hadc;
}

static uint16_t ADC_ReadAveraged(void)
{
    uint32_t sum = 0;
    const int samples = 16;

    for (int i = 0; i < samples; i++)
    {
        HAL_ADC_Start(s_hadc);
        HAL_ADC_PollForConversion(s_hadc, 10);
        sum += HAL_ADC_GetValue(s_hadc);
        HAL_ADC_Stop(s_hadc);
    }

    return (uint16_t)(sum / samples);
}

float Thermistor_ReadTemperatureC(void)
{
    uint16_t adc = ADC_ReadAveraged();

    // Convert ADC to voltage
    float v_ntc = (adc / ADC_MAX_VALUE) * NTC_SUPPLY_V;

    // Protect against divide-by-zero
    if (v_ntc <= 0.0f) v_ntc = 0.0001f;
    if (v_ntc >= NTC_SUPPLY_V) v_ntc = NTC_SUPPLY_V - 0.0001f;

    // Divider: 3.3V -- Rseries -- node -- NTC -- GND
    float r_ntc = (v_ntc * NTC_SERIES_R) / (NTC_SUPPLY_V - v_ntc);

    // Beta equation
    float t0_k = NTC_T0_C + 273.15f;
    float inv_T = (1.0f / t0_k) + (1.0f / NTC_BETA) * logf(r_ntc / NTC_R0);
    float temp_k = 1.0f / inv_T;
    float temp_c = temp_k - 273.15f;

    return temp_c;
}
