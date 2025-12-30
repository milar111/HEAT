#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

// ================== CONTROL MODES ==================
typedef enum {
    CONTROL_MODE_PID   = 0,
    CONTROL_MODE_PULSE = 1
} ControlMode;

// ================== RUN STATES ==================
typedef enum {
    RUN_STATE_IDLE    = 0,
    RUN_STATE_HEATING = 1,
    RUN_STATE_HOLDING = 2,
    RUN_STATE_COOLING = 3,
    RUN_STATE_DONE    = 4
} RunState;

// ================== PROFILE PHASES (NEW) ==================
// These are sub-phases inside RUN_STATE_HEATING / RUN_STATE_HOLDING when profile is enabled.
typedef enum {
    PROFILE_PHASE_LEGACY   = 0,  // profile disabled
    PROFILE_PHASE_PREHEAT  = 1,
    PROFILE_PHASE_SOAK     = 2,
    PROFILE_PHASE_RAMP     = 3,
    PROFILE_PHASE_REFLOW   = 4,  // RUN_STATE_HOLDING (TAL)
    PROFILE_PHASE_COOL     = 5,  // RUN_STATE_COOLING
    PROFILE_PHASE_DONE     = 6   // RUN_STATE_DONE
} ProfilePhase;

// ================== PUBLIC API ==================

void   Control_Init(uint32_t nowMs);
void   Control_Update(uint32_t nowMs);

// Start / Stop
// SetRunning(1) starts the cycle (Heat -> Hold -> Cool)
// SetRunning(0) aborts everything to IDLE
void    Control_SetRunning(uint8_t run);
uint8_t Control_IsRunning(void); // Returns 1 if Heating or Holding

// Check specific states
RunState Control_GetRunState(void);
uint8_t  Control_IsCoolingDown(void); // Returns 1 if in COOLING

// Setpoint (manual / legacy)
float  Control_GetSetpointC(void);
void   Control_SetSetpointC(float sp);

// Active setpoint (what the controller is currently targeting)
float  Control_GetActiveSetpointC(void);

// Profile phase for UI (NEW)
ProfilePhase Control_GetProfilePhase(void);

// Timer Configuration (LEGACY minutes API - preserved)
void    Control_SetHoldTimeMinutes(uint8_t mins);
uint8_t Control_GetHoldTimeMinutes(void);

// Timer Configuration (NEW 15s steps API)
void     Control_SetHoldTime15s(uint16_t steps15s);
uint16_t Control_GetHoldTime15s(void);

// For UI display (seconds remaining in HOLDING)
uint32_t Control_GetRemainingHoldTimeSeconds(void);

// Temps
float  Control_GetTempRawC(void);
float  Control_GetTempFilteredC(void);

// Power
float  Control_GetHeaterPowerPct(void);

// Errors
uint8_t Control_GetSensorError(void);
uint8_t Control_GetOverTempError(void);

// Tuning
void   Control_SetPID(float kp, float ki, float kd);
void   Control_GetPID(float *kp, float *ki, float *kd);

// Safety
void   Control_SetMaxTempCutoff(float tMax);
float  Control_GetMaxTempCutoff(void);

// Mode
void        Control_SetMode(ControlMode m);
ControlMode Control_GetMode(void);

// ================== REFLOW PROFILE (NEW) ==================
// Profile runs inside RUN_STATE_HEATING, then uses RUN_STATE_HOLDING as TAL dwell.

void    Control_SetProfileEnabled(uint8_t en);
uint8_t Control_GetProfileEnabled(void);

// Preheat
void     Control_SetPreheatTempC(float t);
float    Control_GetPreheatTempC(void);
void     Control_SetPreheatTime15s(uint16_t steps15s);
uint16_t Control_GetPreheatTime15s(void);

// Soak
void     Control_SetSoakTempC(float t);
float    Control_GetSoakTempC(void);
void     Control_SetSoakTime15s(uint16_t steps15s);
uint16_t Control_GetSoakTime15s(void);

// Peak
void   Control_SetReflowPeakTempC(float t);
float  Control_GetReflowPeakTempC(void);

// TAL (Time Above Liquidus) -> this is the HOLD timer when profile enabled
void     Control_SetTalTime15s(uint16_t steps15s);
uint16_t Control_GetTalTime15s(void);

#endif // CONTROL_H
