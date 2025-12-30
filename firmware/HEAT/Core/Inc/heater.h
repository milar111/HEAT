#ifndef HEATER_H
#define HEATER_H

#include <stdint.h>

/**
 * @brief Initialize heater timing state and force SSR OFF.
 * @param nowMs  Current tick in ms (use HAL_GetTick()).
 */
void Heater_Init(uint32_t nowMs);

/**
 * @brief Time-proportional control of the SSR.
 *
 * - Uses a fixed time window (HEATER_WINDOW_MS) and turns the SSR
 * ON for a fraction of that window proportional to heaterPowerPct.
 * - If sensorError or overTempError are non-zero, the SSR is
 * immediately turned OFF and the window is reset.
 *
 * @param nowMs          Current tick in ms (HAL_GetTick()).
 * @param heaterPowerPct Desired heater power in percent [0..100].
 * @param sensorError    1 if sensor is failed/out-of-range.
 * @param overTempError  1 if filtered temp above MAX_TEMP_CUTOFF.
 */
void Heater_Update(uint32_t nowMs,
                   float    heaterPowerPct,
                   uint8_t  sensorError,
                   uint8_t  overTempError);

#endif // HEATER_H
