#ifndef FAN_H
#define FAN_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

// called from main loop
// Added 'isCoolingDown' parameter: if 1, fan runs at 100%
void   Fan_Update(float setpointC,
                  float tempFilteredC,
                  float heaterPowerPct,
                  uint32_t nowMs,
                  uint8_t sensorError,
                  uint8_t overTempError,
                  uint8_t isCoolingDown);

// called from TIM2 ISR context
void   Fan_TimerISR(void);

// for UI
uint8_t Fan_GetCurrentPercent(void);


#endif
