#include "fan.h"
#include "stm32f1xx_hal.h"

// ----- hardware mapping -----
#define FAN_GPIO_PORT       GPIOA
#define FAN_GPIO_PIN        GPIO_PIN_5

// ----- fan behavior config -----
#define FAN_MIN_RUN_DUTY      40      // minimum effective duty that actually spins fan
#define FAN_START_BOOST_MS    200     // how long we boost to 100% when starting

// ----- freeze / safety detection -----
#define FREEZE_TEMP_THRESHOLD   100.0f   // above this temp we care about freezing
#define FREEZE_TIME_MS          500u     // 0.5 s
#define FREEZE_EPS_C            0.5f     // minimal change to consider temp "moving"

// Software PWM: 10 kHz ISR, 100-step (100 Hz) PWM
static volatile uint8_t g_fanDuty   = 0;   // 0..100 %
static volatile uint8_t g_pwmPhase  = 0;   // 0..99

// Higher-level control state
static uint8_t  g_fanCurrentPct   = 0;       // what we're actually sending to PWM
static uint8_t  g_fanTargetPct    = 0;       // what control logic wants
static uint32_t g_fanBoostEndTick = 0;       // when to end start-boost

// Freeze detection state
static float    g_lastTempForFreeze   = 25.0f;
static uint32_t g_lastTempChangeMs    = 0;

// ===== ISR-level PWM engine =====
void Fan_TimerISR(void)
{
    g_pwmPhase++;
    if (g_pwmPhase >= 100)
        g_pwmPhase = 0;

    if (g_pwmPhase < g_fanDuty)
    {
        HAL_GPIO_WritePin(FAN_GPIO_PORT, FAN_GPIO_PIN, GPIO_PIN_SET);   // Fan ON
    }
    else
    {
        HAL_GPIO_WritePin(FAN_GPIO_PORT, FAN_GPIO_PIN, GPIO_PIN_RESET); // Fan OFF
    }
}

// ===== helper used by main-level logic =====
static void Fan_SetPWMDuty(uint8_t percent)
{
    if (percent > 100) percent = 100;
    g_fanDuty = percent;
}

// ===== main-level fan logic =====
void Fan_Update(float setpointC,
                float tempFilteredC,
                float heaterPowerPct,
                uint32_t nowMs,
                uint8_t sensorError,
                uint8_t overTempError,
                uint8_t isCoolingDown)
{
    // ----- initialize freeze tracking on first call -----
    if (g_lastTempChangeMs == 0)
    {
        g_lastTempChangeMs  = nowMs;
        g_lastTempForFreeze = tempFilteredC;
    }

    // Track temperature movement
    float diff = tempFilteredC - g_lastTempForFreeze;
    if (diff < 0.0f) diff = -diff;

    if (diff > FREEZE_EPS_C)
    {
        // temp has moved enough -> update reference
        g_lastTempForFreeze = tempFilteredC;
        g_lastTempChangeMs  = nowMs;
    }

    // ----- compute EMERGENCY / FORCED COOL condition -----
    uint8_t forceFull = 0;

    // 1) Explicit cooling phase requested by Control
    if (isCoolingDown)
    {
        forceFull = 1;
    }

    // 2) any error flags from control
    if (sensorError || overTempError)
    {
        forceFull = 1;
    }

    // 3) absurd temp values
    if (tempFilteredC < 0.0f || tempFilteredC > 300.0f)
    {
        forceFull = 1;
    }

    // 4) freeze detection: high temp + no change
    if (!forceFull &&
        tempFilteredC > FREEZE_TEMP_THRESHOLD &&
        (nowMs - g_lastTempChangeMs) > FREEZE_TIME_MS)
    {
        forceFull = 1;
    }

    if (forceFull)
    {
        // Force full fan, ignore normal logic
        g_fanTargetPct = 100;
    }
    else
    {
        // ===== normal PID-aware fan logic (Only when NOT cooling down) =====

        // Basic idea:
        // - Below setpoint - 5°C: fan off
        // - From setpoint to setpoint + 60°C: ramp 0..100%
        // - Above setpoint + 60°C: clamp at 100%
        // - Reduce fan when heater is still strongly ON to avoid fighting it

        float deltaT = tempFilteredC - setpointC;
        float fanBase = 0.0f;

        if (deltaT <= -5.0f)
        {
            // Quite below setpoint -> no cooling
            fanBase = 0.0f;
        }
        else if (deltaT <= 0.0f)
        {
            // Within [-5, 0] around setpoint -> small cooling (optional)
            fanBase = 10.0f;   // set to 0.0f if you want no fan here
        }
        else
        {
            // Above setpoint: ramp 0..100% over deltaT = 0..60°C
            if (deltaT >= 60.0f)
                fanBase = 100.0f;
            else
                fanBase = (deltaT / 60.0f) * 100.0f;
        }

        // Merge with heater power: if heater is still working hard (>50%),
        // reduce fan to avoid fighting it too much.
        // factor = 1.0 when heater off, ~0.5 when heater at 100%
        float heaterFactor = 1.0f - 0.5f * (heaterPowerPct / 100.0f);
        if (heaterFactor < 0.2f) heaterFactor = 0.2f;  // don't kill it completely

        float fanTargetFloat = fanBase * heaterFactor;

        // Clamp
        if (fanTargetFloat < 0.0f) fanTargetFloat = 0.0f;
        if (fanTargetFloat > 100.0f) fanTargetFloat = 100.0f;

        g_fanTargetPct = (uint8_t)(fanTargetFloat + 0.5f); // round
    }

    // ----- Apply min duty + start boost -----
    if (g_fanTargetPct == 0)
    {
        // Target is off -> no boost, no min duty
        g_fanCurrentPct   = 0;
        g_fanBoostEndTick = 0;
    }
    else
    {
        // We want some fan speed
        uint8_t targetWithMin = g_fanTargetPct;

        // Enforce minimum running duty if not zero
        if (targetWithMin > 0 && targetWithMin < FAN_MIN_RUN_DUTY)
            targetWithMin = FAN_MIN_RUN_DUTY;

        // Detect 0 -> >0 transition for start boost
        if (g_fanCurrentPct == 0 && targetWithMin > 0)
        {
            // Just turned on -> boost to 100% for a short time
            g_fanCurrentPct   = 100;
            g_fanBoostEndTick = nowMs + FAN_START_BOOST_MS;
        }
        else
        {
            // If we're still in boost period, keep 100%
            if (g_fanBoostEndTick != 0 && nowMs < g_fanBoostEndTick)
            {
                g_fanCurrentPct = 100;
            }
            else
            {
                // Boost ended -> go to target-with-min
                g_fanCurrentPct = targetWithMin;
            }
        }
    }

    // Finally, send to PWM engine
    Fan_SetPWMDuty(g_fanCurrentPct);
}

uint8_t Fan_GetCurrentPercent(void)
{
    return g_fanCurrentPct;
}
