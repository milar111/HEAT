#include "heater.h"
#include "stm32f1xx_hal.h"

// ---- hardware mapping ----
// PB6 -> MOC3063 input (SSR control)
#define HEATER_GPIO_PORT    GPIOB
#define HEATER_GPIO_PIN     GPIO_PIN_6

// Time-proportional control window (ms)
// 250 ms = 4 updates per second, good with zero-cross SSR on 50 Hz mains
#define HEATER_WINDOW_MS    250u

// Internal state: start of current window, and ON time inside that window
static uint32_t g_windowStartMs = 0;
static uint32_t g_windowOnMs    = 0;

/**
 * @brief Initialize heater timing state and force SSR OFF.
 */
void Heater_Init(uint32_t nowMs)
{
    g_windowStartMs = nowMs;
    g_windowOnMs    = 0;

    // Start with heater OFF
    HAL_GPIO_WritePin(HEATER_GPIO_PORT, HEATER_GPIO_PIN, GPIO_PIN_RESET);
}

/**
 * @brief Time-proportional SSR control with safety.
 */
void Heater_Update(uint32_t nowMs,
                   float    heaterPowerPct,
                   uint8_t  sensorError,
                   uint8_t  overTempError)
{
    // --------- 1) SAFETY: any error => hard OFF ----------
    if (sensorError || overTempError)
    {
        // Immediately disable SSR
        HAL_GPIO_WritePin(HEATER_GPIO_PORT, HEATER_GPIO_PIN, GPIO_PIN_RESET);

        // Force this and next window to 0% ON
        g_windowOnMs    = 0;
        g_windowStartMs = nowMs;

        return;
    }

    // --------- 2) Start a new window when needed ----------
    uint32_t elapsed = nowMs - g_windowStartMs;
    if (elapsed >= HEATER_WINDOW_MS)
    {
        // New window starts now
        g_windowStartMs = nowMs;
        elapsed         = 0;

        // Clamp requested power
        if (heaterPowerPct < 0.0f)   heaterPowerPct = 0.0f;
        if (heaterPowerPct > 100.0f) heaterPowerPct = 100.0f;

        // Compute ON time (ms) inside this window
        g_windowOnMs = (uint32_t)((heaterPowerPct / 100.0f) *
                                  (float)HEATER_WINDOW_MS + 0.5f);
    }

    // --------- 3) Apply ON/OFF inside the current window ----------
    if (g_windowOnMs == 0)
    {
        // 0% power
        HAL_GPIO_WritePin(HEATER_GPIO_PORT, HEATER_GPIO_PIN, GPIO_PIN_RESET);
    }
    else if (g_windowOnMs >= HEATER_WINDOW_MS)
    {
        // 100% power
        HAL_GPIO_WritePin(HEATER_GPIO_PORT, HEATER_GPIO_PIN, GPIO_PIN_SET);
    }
    else
    {
        // Partial duty: ON for [0 .. g_windowOnMs), then OFF
        if (elapsed < g_windowOnMs)
        {
            HAL_GPIO_WritePin(HEATER_GPIO_PORT, HEATER_GPIO_PIN, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(HEATER_GPIO_PORT, HEATER_GPIO_PIN, GPIO_PIN_RESET);
        }
    }
}
