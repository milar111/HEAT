#include "control.h"
#include "thermistor.h"

#include <math.h>

// =========================================================
//                       CONSTANTS
// =========================================================
#define CONTROL_PERIOD_MS        100u
#define SENSOR_FAIL_TEMP_MIN    -20.0f
#define SENSOR_FAIL_TEMP_MAX    300.0f
#define DEFAULT_SETPOINT_C       150.0f
#define DEFAULT_MAX_TEMP_CUTOFF  300.0f
#define DEFAULT_HOLD_TIME_MINS   5u
#define TEMP_FILTER_TC_MS        100.0f
#define STEP_15S_MS              15000u

// Behaviour tuning
#define TARGET_TOLERANCE_C         1.0f
#define COOL_DONE_TEMP_C          40.0f
#define MIN_TARGET_C              20.0f
#define CUTOFF_MARGIN_C           10.0f

// Smart pulse configuration
#define ZONE_MID_PCT             0.70f
#define ZONE_NEAR_PCT            0.85f
#define PULSE_MS_FAR             2000u
#define PULSE_MS_MID              500u
#define PULSE_MS_NEAR             250u
#define COAST_MIN_WAIT_MS        3000u
#define COAST_MAX_WAIT_MS       20000u
#define COAST_DROP_THRESHOLD      0.15f

// Simple PID limits
#define PID_OUTPUT_MIN             0.0f
#define PID_OUTPUT_MAX           100.0f
#define PID_INTEGRAL_MAX        1000.0f

// =========================================================
//                    RUNTIME STRUCTURES
// =========================================================
typedef struct
{
    float      setpointC;
    float      targetC;
    float      tempRawC;
    float      tempFilteredC;
    float      heaterPowerPct;
    float      maxTempCutoff;
    uint8_t    sensorError;
    uint8_t    sensorResumePending;
    RunState   resumeState;
    uint8_t    overTempError;
    uint32_t   lastTick;
    ControlMode mode;
    RunState    runState;
    uint16_t    holdTime15s;
    uint32_t    holdStartTick;
} ControlContext;

static ControlContext g_ctx = {
    .setpointC       = DEFAULT_SETPOINT_C,
    .targetC         = DEFAULT_SETPOINT_C,
    .tempRawC        = 25.0f,
    .tempFilteredC   = 25.0f,
    .heaterPowerPct  = 0.0f,
    .maxTempCutoff   = DEFAULT_MAX_TEMP_CUTOFF,
    .sensorError     = 0,
    .sensorResumePending = 0,
    .resumeState     = RUN_STATE_IDLE,
    .overTempError   = 0,
    .lastTick        = 0,
    .mode            = CONTROL_MODE_PULSE,
    .runState        = RUN_STATE_IDLE,
    .holdTime15s     = (uint16_t)(DEFAULT_HOLD_TIME_MINS * 4u),
    .holdStartTick   = 0
};

// -------- Smart pulse state --------
typedef enum {
    SP_STATE_DECIDE = 0,
    SP_STATE_PULSE,
    SP_STATE_COAST
} SmartPulseState;

typedef struct
{
    SmartPulseState state;
    uint32_t        timerStart;
    uint32_t        pulseWidth;
    float           peakTemp;
} SmartPulseContext;

static SmartPulseContext g_sp = {
    .state      = SP_STATE_DECIDE,
    .timerStart = 0,
    .pulseWidth = 0,
    .peakTemp   = 0.0f
};

// -------- PID state --------
typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float lastError;
} PIDContext;

static PIDContext g_pid = {
    .kp         = 10.0f,
    .ki         = 0.5f,
    .kd         = 0.0f,
    .integral   = 0.0f,
    .lastError  = 0.0f
};

// -------- Profile state --------
typedef enum {
    REFLOW_PHASE_PREHEAT = 0,
    REFLOW_PHASE_SOAK,
    REFLOW_PHASE_RAMP
} ReflowPhase;

typedef struct
{
    uint8_t     enabled;
    float       preheatTempC;
    uint16_t    preheatTime15s;
    float       soakTempC;
    uint16_t    soakTime15s;
    float       reflowPeakC;
    uint16_t    talTime15s;
    ReflowPhase phase;
    uint8_t     reachedTemp;
    uint32_t    phaseStartTick;
} ProfileContext;

static ProfileContext g_prof;

// =========================================================
//                        HELPERS
// =========================================================
static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void Control_ResetOutputs(void)
{
    g_ctx.heaterPowerPct = 0.0f;
    g_sp.state           = SP_STATE_DECIDE;
    g_sp.pulseWidth      = 0;
}

// ============================================
//         REFLOW PROFILE (NEW)
// ============================================
typedef enum {
    REFLOW_PHASE_PREHEAT = 0,
    REFLOW_PHASE_SOAK    = 1,
    REFLOW_PHASE_RAMP    = 2
} ReflowPhase;

typedef struct {
    uint8_t  enabled;

    float    preheatTempC;
    uint16_t preheatTime15s;

    float    soakTempC;
    uint16_t soakTime15s;

    float    reflowPeakC;
    uint16_t talTime15s; // maps to HOLD timer
} ReflowProfile;

static ReflowProfile g_prof;
static ReflowPhase   g_phase            = REFLOW_PHASE_PREHEAT;
static uint32_t      g_phaseStartTick   = 0;
static uint8_t       g_phaseReachedTemp = 0;

// -------------------------------------------------
// Helpers
// -------------------------------------------------
static void Control_AbortToIdle(void)
{
    g_heaterPowerPct = 0.0f;
    g_runState       = RUN_STATE_IDLE;
    g_sp_state       = SP_STATE_DECIDE;
}

static void Control_Profile_LoadDefaults(void)
{
    g_prof.enabled        = 0;       // default OFF => legacy behavior remains default

    g_prof.preheatTempC   = 150.0f;
    g_prof.preheatTime15s = 6;       // 90s

    g_prof.soakTempC      = 180.0f;
    g_prof.soakTime15s    = 6;       // 90s

    g_prof.reflowPeakC    = 240.0f;
    g_prof.talTime15s     = 3;       // 45s
}

static void Control_Profile_OnStart(uint32_t nowMs)
{
    g_phase            = REFLOW_PHASE_PREHEAT;
    g_phaseStartTick   = nowMs;
    g_phaseReachedTemp = 0;
    g_sp_state         = SP_STATE_DECIDE;
}

static float Control_ClampTarget(float target)
{
    if (target > (g_maxTempCutoff - 10.0f)) target = g_maxTempCutoff - 10.0f;
    if (target < 20.0f) target = 20.0f;
    return target;
}

static uint8_t Control_ReadAndFilterTemp(float dt_s)
{
    g_tempRawC = Thermistor_ReadTemperatureC();

    // Sensor error handling
    if (g_tempRawC < SENSOR_FAIL_TEMP_MIN || g_tempRawC > SENSOR_FAIL_TEMP_MAX) {
        g_sensorError    = 1;
        Control_AbortToIdle();
        return 0;
    }
    g_sensorError = 0;

    // Filter
    float tc_s  = TEMP_FILTER_TC_MS / 1000.0f;
    float alpha = dt_s / (tc_s + dt_s);
    g_tempFilteredC = g_tempFilteredC + alpha * (g_tempRawC - g_tempFilteredC);

    // Overtemp cutoff
    if (g_tempFilteredC > g_maxTempCutoff) {
        g_overTempError   = 1;
        Control_AbortToIdle();
        return 0;
    }
    g_overTempError = 0;

    return 1;
}

static void Control_Update_SmartPulse(uint32_t nowMs)
{
    g_prof.enabled        = 0;
    g_prof.preheatTempC   = 150.0f;
    g_prof.preheatTime15s = 6;   // 90 s
    g_prof.soakTempC      = 180.0f;
    g_prof.soakTime15s    = 6;   // 90 s
    g_prof.reflowPeakC    = 240.0f;
    g_prof.talTime15s     = 3;   // 45 s

    g_prof.phase          = REFLOW_PHASE_PREHEAT;
    g_prof.reachedTemp    = 0;
    g_prof.phaseStartTick = nowMs;
}

static void Control_ResetCycle(uint32_t nowMs)
{
    g_ctx.runState      = RUN_STATE_IDLE;
    g_ctx.holdStartTick = 0;
    g_ctx.sensorResumePending = 0;
    g_prof.phase        = REFLOW_PHASE_PREHEAT;
    g_prof.reachedTemp  = 0;
    g_prof.phaseStartTick = nowMs;
    Control_ResetOutputs();
}

static float Control_ClampTarget(float target)
{
    float hi = g_ctx.maxTempCutoff - CUTOFF_MARGIN_C;
    if (hi < MIN_TARGET_C) hi = MIN_TARGET_C;
    target = clampf(target, MIN_TARGET_C, hi);
    return target;
}

static uint8_t Control_UpdateTemperature(uint32_t nowMs, float dt_s)
{
    (void)nowMs;
    g_ctx.tempRawC = Thermistor_ReadTemperatureC();

    if (g_ctx.tempRawC < SENSOR_FAIL_TEMP_MIN || g_ctx.tempRawC > SENSOR_FAIL_TEMP_MAX)
    {
        g_ctx.sensorError = 1;
        g_ctx.sensorResumePending = 1;
        g_ctx.resumeState = g_ctx.runState;
        Control_ResetOutputs();
        g_ctx.runState = RUN_STATE_IDLE;
        return 0;
    }

    if (g_ctx.sensorError && g_ctx.sensorResumePending)
    {
        g_ctx.runState            = g_ctx.resumeState;
        g_ctx.sensorResumePending = 0;
    }

    g_ctx.sensorError = 0;

    // Simple first-order filter
    float tc_s  = TEMP_FILTER_TC_MS / 1000.0f;
    float alpha = dt_s / (tc_s + dt_s);
    g_ctx.tempFilteredC += alpha * (g_ctx.tempRawC - g_ctx.tempFilteredC);

    if (g_ctx.tempFilteredC > g_ctx.maxTempCutoff)
    {
        g_ctx.overTempError = 1;
        Control_ResetOutputs();
        g_ctx.runState = RUN_STATE_IDLE;
        return 0;
    }

    g_ctx.overTempError = 0;
    return 1;
}

// =========================================================
//                       CONTROL LOOPS
// =========================================================
static float Control_RunPID(float targetC, float tempC, float dt_s)
{
    float error = targetC - tempC;

    g_pid.integral += error * dt_s * g_pid.ki;
    g_pid.integral = clampf(g_pid.integral, -PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);

    float deriv = (error - g_pid.lastError) / dt_s;
    g_pid.lastError = error;

    float output = g_pid.kp * error + g_pid.integral + g_pid.kd * deriv;
    return clampf(output, PID_OUTPUT_MIN, PID_OUTPUT_MAX);
}

static void Control_RunSmartPulse(uint32_t nowMs)
{
    float temp   = g_ctx.tempFilteredC;
    float target = g_ctx.targetC;

    if (temp >= target)
    {
        Control_ResetOutputs();
        return;
    }

    switch (g_sp.state)
    {
        case SP_STATE_DECIDE:
        {
            float ratio = temp / target;
            if (ratio < ZONE_MID_PCT)       g_sp.pulseWidth = PULSE_MS_FAR;
            else if (ratio < ZONE_NEAR_PCT) g_sp.pulseWidth = PULSE_MS_MID;
            else                            g_sp.pulseWidth = PULSE_MS_NEAR;

            if (ratio < ZONE_MID_PCT)       g_sp_pulseWidth = PULSE_MS_FAR;
            else if (ratio < ZONE_NEAR_PCT) g_sp_pulseWidth = PULSE_MS_MID;
            else                            g_sp_pulseWidth = PULSE_MS_NEAR;

            g_sp_timerStart   = nowMs;
            g_heaterPowerPct  = 100.0f;
            g_sp_state        = SP_STATE_PULSE;
        } break;

        case SP_STATE_PULSE:
        {
            if (temp >= target)
            {
                Control_ResetOutputs();
                return;
            }

            if ((nowMs - g_sp.timerStart) >= g_sp.pulseWidth)
            {
                g_ctx.heaterPowerPct = 0.0f;
                g_sp.peakTemp        = temp;
                g_sp.timerStart      = nowMs;
                g_sp.state           = SP_STATE_COAST;
            }
        } break;

        case SP_STATE_COAST:
        {
            g_ctx.heaterPowerPct = 0.0f;

            if (temp > g_sp.peakTemp) g_sp.peakTemp = temp;

            uint32_t coastTime = nowMs - g_sp.timerStart;
            if (coastTime < COAST_MIN_WAIT_MS) return;

            float drop = g_sp.peakTemp - temp;
            if (drop >= COAST_DROP_THRESHOLD)
            {
                g_sp.state = SP_STATE_DECIDE;
                return;
            }

            if (coastTime >= COAST_MAX_WAIT_MS)
            {
                g_sp.state = SP_STATE_DECIDE;
            }
        } break;
    }
}

// Called only while RUN_STATE_HEATING and profile enabled.
static float Control_Profile_UpdateAndGetTarget(uint32_t nowMs)
{
    float target = g_prof.reflowPeakC;
    float temp   = g_ctx.tempFilteredC;

    switch (g_prof.phase)
    {
        case REFLOW_PHASE_PREHEAT:
        {
            target = g_prof.preheatTempC;
            if (!g_prof.reachedTemp && temp >= (target - TARGET_TOLERANCE_C))
            {
                g_prof.reachedTemp   = 1;
                g_prof.phaseStartTick = nowMs;
            }
            else if (g_prof.reachedTemp)
            {
                uint32_t elapsed = nowMs - g_prof.phaseStartTick;
                uint32_t limit   = (uint32_t)g_prof.preheatTime15s * STEP_15S_MS;
                if (elapsed >= limit)
                {
                    g_prof.phase        = REFLOW_PHASE_SOAK;
                    g_prof.reachedTemp  = 0;
                    g_prof.phaseStartTick = nowMs;
                    g_sp.state          = SP_STATE_DECIDE;
                }
            }
        } break;

        case REFLOW_PHASE_SOAK:
        {
            target = g_prof.soakTempC;
            if (!g_prof.reachedTemp && temp >= (target - TARGET_TOLERANCE_C))
            {
                g_prof.reachedTemp   = 1;
                g_prof.phaseStartTick = nowMs;
            }
            else if (g_prof.reachedTemp)
            {
                uint32_t elapsed = nowMs - g_prof.phaseStartTick;
                uint32_t limit   = (uint32_t)g_prof.soakTime15s * STEP_15S_MS;
                if (elapsed >= limit)
                {
                    g_prof.phase        = REFLOW_PHASE_RAMP;
                    g_prof.reachedTemp  = 0;
                    g_prof.phaseStartTick = nowMs;
                    g_sp.state          = SP_STATE_DECIDE;
                }
            }
        } break;

        case REFLOW_PHASE_RAMP:
        default:
        {
            target = g_prof.reflowPeakC;
            if (temp >= (target - TARGET_TOLERANCE_C))
            {
                Control_StartHold(nowMs);
            }
        } break;
    }

    return Control_ClampTarget(target);
}

static void Control_HandleHeating(uint32_t nowMs)
{
    if (g_prof.enabled) g_targetC = Control_Profile_UpdateAndGetTarget(nowMs);
    else               g_targetC = g_setpointC;

    g_targetC = Control_ClampTarget(g_targetC);

    Control_Update_SmartPulse(nowMs);

    if (g_tempFilteredC >= (g_targetC - 1.0f))
    {
        if (g_prof.enabled && (g_phase != REFLOW_PHASE_RAMP)) {
            // ignore; preheat/soak targets reached are handled by profile
            return;
        }

        g_runState      = RUN_STATE_HOLDING;
        g_holdStartTick = nowMs;

        if (g_prof.enabled) {
            g_holdTime15s = g_prof.talTime15s;
        }
    }
}

static void Control_HandleHolding(uint32_t nowMs)
{
    if (g_prof.enabled) g_targetC = g_prof.reflowPeakC;
    else               g_targetC = g_setpointC;

    g_targetC = Control_ClampTarget(g_targetC);
    Control_Update_SmartPulse(nowMs);

    if (g_holdTime15s > 0)
    {
        uint32_t elapsedMs = nowMs - g_holdStartTick;
        uint32_t limitMs   = (uint32_t)g_holdTime15s * STEP_15S_MS;
        if (elapsedMs >= limitMs) {
            g_runState = RUN_STATE_COOLING;
        }
    }
}

static void Control_HandleCooling(void)
{
    g_heaterPowerPct = 0.0f;
    g_targetC        = g_setpointC;

    if (g_tempFilteredC < 40.0f) {
        g_runState = RUN_STATE_DONE;
    }
}

// =========================================================
//                     STATE HANDLERS
// =========================================================
static void Control_HandleHeating(uint32_t nowMs, float dt_s)
{
    if (g_prof.enabled)
    {
        g_ctx.targetC = Control_ProfileStep(nowMs);
    }
    else
    {
        g_ctx.targetC = Control_ClampTarget(g_ctx.setpointC);
    }

    Control_ApplyOutput(nowMs, dt_s);

    if (!g_prof.enabled && g_ctx.tempFilteredC >= (g_ctx.targetC - TARGET_TOLERANCE_C))
    {
        Control_StartHold(nowMs);
    }
}

static void Control_HandleHolding(uint32_t nowMs, float dt_s)
{
    g_ctx.targetC = g_prof.enabled ? Control_ClampTarget(g_prof.reflowPeakC)
                                   : Control_ClampTarget(g_ctx.setpointC);

    Control_ApplyOutput(nowMs, dt_s);

    if (!Control_ReadAndFilterTemp(dt_s)) return;

    if (!Control_UpdateTemperature(nowMs, dt_s)) return;

    switch (g_ctx.runState)
    {
        case RUN_STATE_IDLE:
            Control_ResetOutputs();
            g_ctx.targetC = Control_ClampTarget(g_ctx.setpointC);
            break;

        case RUN_STATE_HEATING:
            Control_HandleHeating(nowMs);
            break;

        case RUN_STATE_HOLDING:
            Control_HandleHolding(nowMs);
            break;

        case RUN_STATE_COOLING:
            Control_HandleCooling();
            break;

        case RUN_STATE_DONE:
            Control_ResetOutputs();
            g_ctx.targetC = Control_ClampTarget(g_ctx.setpointC);
            break;
    }
}

void Control_SetRunning(uint8_t run)
{
    if (run)
    {
        if (g_ctx.runState == RUN_STATE_IDLE || g_ctx.runState == RUN_STATE_DONE)
        {
            g_ctx.runState      = RUN_STATE_HEATING;
            g_ctx.holdStartTick = 0;
            g_sp.state          = SP_STATE_DECIDE;
            g_pid.integral      = 0.0f;
            g_pid.lastError     = 0.0f;

            if (g_prof.enabled)
            {
                g_prof.phase          = REFLOW_PHASE_PREHEAT;
                g_prof.reachedTemp    = 0;
                g_prof.phaseStartTick = g_ctx.lastTick;
            }
        }
    } else {
        Control_AbortToIdle();
    }
}

uint8_t Control_IsRunning(void)
{
    return (g_ctx.runState == RUN_STATE_HEATING || g_ctx.runState == RUN_STATE_HOLDING);
}

RunState Control_GetRunState(void)
{
    return g_ctx.runState;
}

uint8_t Control_IsCoolingDown(void)
{
    return (g_ctx.runState == RUN_STATE_COOLING);
}

ProfilePhase Control_GetProfilePhase(void)
{
    if (!g_prof.enabled)
    {
        if (g_ctx.runState == RUN_STATE_COOLING) return PROFILE_PHASE_COOL;
        if (g_ctx.runState == RUN_STATE_DONE)    return PROFILE_PHASE_DONE;
        return PROFILE_PHASE_LEGACY;
    }

    if (g_ctx.runState == RUN_STATE_HEATING)
    {
        if (g_prof.phase == REFLOW_PHASE_PREHEAT) return PROFILE_PHASE_PREHEAT;
        if (g_prof.phase == REFLOW_PHASE_SOAK)    return PROFILE_PHASE_SOAK;
        return PROFILE_PHASE_RAMP;
    }

    if (g_ctx.runState == RUN_STATE_HOLDING) return PROFILE_PHASE_REFLOW;
    if (g_ctx.runState == RUN_STATE_COOLING) return PROFILE_PHASE_COOL;
    if (g_ctx.runState == RUN_STATE_DONE)    return PROFILE_PHASE_DONE;

    return PROFILE_PHASE_LEGACY;
}

void Control_SetHoldTime15s(uint16_t steps15s)
{
    if (steps15s > 240u) steps15s = 240u; // max 60 minutes
    g_ctx.holdTime15s = steps15s;
}

uint16_t Control_GetHoldTime15s(void)
{
    return g_ctx.holdTime15s;
}

void Control_SetHoldTimeMinutes(uint8_t mins)
{
    Control_SetHoldTime15s((uint16_t)mins * 4u);
}

uint8_t Control_GetHoldTimeMinutes(void)
{
    return (uint8_t)((g_ctx.holdTime15s + 2u) / 4u);
}

uint32_t Control_GetRemainingHoldTimeSeconds(void)
{
    if (g_ctx.runState != RUN_STATE_HOLDING) return 0;

    uint32_t elapsed = g_ctx.lastTick - g_ctx.holdStartTick;
    uint32_t limit   = (uint32_t)g_ctx.holdTime15s * STEP_15S_MS;
    if (elapsed >= limit) return 0;

    return (limit - elapsed) / 1000u;
}

// ----- getters / setters -----
float Control_GetSetpointC(void) { return g_ctx.setpointC; }
void  Control_SetSetpointC(float sp) { g_ctx.setpointC = sp; }
float Control_GetActiveSetpointC(void) { return g_ctx.targetC; }
float Control_GetTempRawC(void)       { return g_ctx.tempRawC; }
float Control_GetTempFilteredC(void)  { return g_ctx.tempFilteredC; }
float Control_GetHeaterPowerPct(void) { return g_ctx.heaterPowerPct; }
uint8_t Control_GetSensorError(void)  { return g_ctx.sensorError; }
uint8_t Control_GetOverTempError(void){ return g_ctx.overTempError; }

void Control_SetPID(float kp, float ki, float kd)
{
    g_pid.kp = kp; g_pid.ki = ki; g_pid.kd = kd;
    g_pid.integral  = 0.0f;
    g_pid.lastError = 0.0f;
}

void Control_GetPID(float *kp, float *ki, float *kd)
{
    if (kp) *kp = g_pid.kp;
    if (ki) *ki = g_pid.ki;
    if (kd) *kd = g_pid.kd;
}

void Control_SetMaxTempCutoff(float tMax) { g_ctx.maxTempCutoff = tMax; }
float Control_GetMaxTempCutoff(void)      { return g_ctx.maxTempCutoff; }

void Control_SetMode(ControlMode m)
{
    g_ctx.mode = m;
    Control_ResetOutputs();
    g_pid.integral  = 0.0f;
    g_pid.lastError = 0.0f;
}
ControlMode Control_GetMode(void) { return g_ctx.mode; }

// ================== REFLOW PROFILE API ==================
void Control_SetProfileEnabled(uint8_t en)
{
    g_prof.enabled = (en ? 1u : 0u);
}

uint8_t Control_GetProfileEnabled(void)
{
    return g_prof.enabled;
}

void Control_SetPreheatTempC(float t) { g_prof.preheatTempC = t; }
float Control_GetPreheatTempC(void)   { return g_prof.preheatTempC; }

void Control_SetPreheatTime15s(uint16_t steps15s)
{
    if (steps15s > 240u) steps15s = 240u;
    g_prof.preheatTime15s = steps15s;
}
uint16_t Control_GetPreheatTime15s(void) { return g_prof.preheatTime15s; }

void Control_SetSoakTempC(float t) { g_prof.soakTempC = t; }
float Control_GetSoakTempC(void)   { return g_prof.soakTempC; }

void Control_SetSoakTime15s(uint16_t steps15s)
{
    if (steps15s > 240u) steps15s = 240u;
    g_prof.soakTime15s = steps15s;
}
uint16_t Control_GetSoakTime15s(void) { return g_prof.soakTime15s; }

void Control_SetReflowPeakTempC(float t) { g_prof.reflowPeakC = t; }
float Control_GetReflowPeakTempC(void)   { return g_prof.reflowPeakC; }

void Control_SetTalTime15s(uint16_t steps15s)
{
    if (steps15s > 240u) steps15s = 240u;
    g_prof.talTime15s = steps15s;
}
uint16_t Control_GetTalTime15s(void) { return g_prof.talTime15s; }
