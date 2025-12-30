/*
 * ui.c
 *
 * Created on: Nov 29, 2025
 * Author: dyord
 */

#include "ui.h"
#include "main.h"
#include "ssd1306.h"
#include "control.h"
#include "fan.h"
#include "storage.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

// --------- UI state ----------
typedef enum {
    UI_SCREEN_STATUS = 0,
    UI_SCREEN_TUNE   = 1,
    UI_SCREEN_GRAPH  = 2
} UiScreen;

static UiScreen g_uiScreen   = UI_SCREEN_STATUS;
static uint8_t  g_uiEditMode = 0;
static uint8_t  g_uiSelected = 0;

// Previous button states
static uint8_t  g_prevB1 = 0;
static uint8_t  g_prevB2 = 0;
static uint8_t  g_prevB3 = 0;

// ---- Temp graph buffer ----
#define GRAPH_WIDTH             128
#define GRAPH_SAMPLE_PERIOD_MS  500u
#define GRAPH_HEIGHT            54
#define GRAPH_Y_OFFSET          10

static float    g_tempHistory[GRAPH_WIDTH];
static uint8_t  g_graphWriteIndex     = 0;
static uint32_t g_lastGraphSampleTick = 0;

// Settings UI layout
#define SETTINGS_ROW_H          8
#define SETTINGS_START_Y        16
#define SETTINGS_LEGEND_Y       56
// We reserve y=56..63 for legend, so available height is 56-16 = 40px => 5 rows of 8px
#define SETTINGS_VISIBLE_ROWS   5

static void FormatMmSsFrom15s(char *out, size_t outSz, uint16_t steps15s)
{
    uint32_t totalSec = (uint32_t)steps15s * 15u;
    uint32_t mm = totalSec / 60u;
    uint32_t ss = totalSec % 60u;
    snprintf(out, outSz, "%02lu:%02lu", (unsigned long)mm, (unsigned long)ss);
}

static const char* PhaseToStr(ProfilePhase ph)
{
    switch (ph)
    {
        case PROFILE_PHASE_PREHEAT: return "PREHEAT";
        case PROFILE_PHASE_SOAK:    return "SOAK";
        case PROFILE_PHASE_RAMP:    return "RAMP";
        case PROFILE_PHASE_REFLOW:  return "REFLOW";
        case PROFILE_PHASE_COOL:    return "COOL";
        case PROFILE_PHASE_DONE:    return "DONE";
        default:                    return "LEGACY";
    }
}

// --------------------------------------------------
// Helper Graphics Functions
// --------------------------------------------------
static void DrawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color)
{
    for (int i = 0; i < w; i++) {
        SSD1306_DrawPixel(x + i, y, color);
        SSD1306_DrawPixel(x + i, y + h - 1, color);
    }
    for (int j = 0; j < h; j++) {
        SSD1306_DrawPixel(x, y + j, color);
        SSD1306_DrawPixel(x + w - 1, y + j, color);
    }
}

static void DrawProgressBar(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t percent)
{
    DrawRect(x, y, w, h, 1);
    if (percent > 100) percent = 100;
    uint8_t fillW = (uint8_t)(((float)(w - 2) * (float)percent) / 100.0f);
    for (int i = 0; i < fillW; i++) {
        for (int j = 1; j < h - 1; j++) {
            SSD1306_DrawPixel(x + 1 + i, y + j, 1);
        }
    }
}

// --------------------------------------------------
// Init
// --------------------------------------------------
void UI_Init(void)
{
    for (int i = 0; i < GRAPH_WIDTH; i++)
        g_tempHistory[i] = 25.0f;

    g_graphWriteIndex     = 0;
    g_lastGraphSampleTick = HAL_GetTick();

    g_uiScreen   = UI_SCREEN_STATUS;
    g_uiEditMode = 0;
    g_uiSelected = 0;

    g_prevB1 = 0;
    g_prevB2 = 0;
    g_prevB3 = 0;
}

// --------------------------------------------------
// UI_Task
// --------------------------------------------------
void UI_Task(uint32_t now)
{
    float   activeSpC = Control_GetActiveSetpointC();
    float   tempF     = Control_GetTempFilteredC();
    float   heaterP   = Control_GetHeaterPowerPct();
    uint8_t errSens   = Control_GetSensorError();
    uint8_t errOver   = Control_GetOverTempError();
    uint8_t isCooling = Control_IsCoolingDown();

    // ===== Graph history =====
    if ((now - g_lastGraphSampleTick) >= GRAPH_SAMPLE_PERIOD_MS)
    {
        g_lastGraphSampleTick = now;
        g_tempHistory[g_graphWriteIndex] = tempF;
        g_graphWriteIndex++;
        if (g_graphWriteIndex >= GRAPH_WIDTH)
            g_graphWriteIndex = 0;
    }

    // ===== Buttons (edge detect) =====
    uint8_t b1_raw = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET);
    uint8_t b2_raw = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET);
    uint8_t b3_raw = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET);

    uint8_t p1 = (b1_raw && !g_prevB1);
    uint8_t p2 = (b2_raw && !g_prevB2);
    uint8_t p3 = (b3_raw && !g_prevB3);

    g_prevB1 = b1_raw;
    g_prevB2 = b2_raw;
    g_prevB3 = b3_raw;

    uint8_t h1 = b1_raw;
    uint8_t h2 = b2_raw;

    // ===== Fan update =====
    Fan_Update(activeSpC, tempF, heaterP, now, errSens, errOver, isCooling);
    uint8_t fanPct = Fan_GetCurrentPercent();

    float kp, ki, kd;
    Control_GetPID(&kp, &ki, &kd);
    float maxT = Control_GetMaxTempCutoff();

    // Profile values
    uint8_t  profEn    = Control_GetProfileEnabled();
    float    preT      = Control_GetPreheatTempC();
    uint16_t pre15s    = Control_GetPreheatTime15s();
    float    soakT     = Control_GetSoakTempC();
    uint16_t soak15s   = Control_GetSoakTime15s();
    float    peakT     = Control_GetReflowPeakTempC();
    uint16_t tal15s    = Control_GetTalTime15s();

    // Legacy hold time (15s base)
    uint16_t hold15s   = Control_GetHoldTime15s();

    // =========================================================
    // Settings menu items order
    // IMPORTANT CHANGE: PID moved to the end (not used currently)
    // =========================================================
    typedef enum {
        ITEM_SETPOINT = 0,
        ITEM_MAXTEMP,

        ITEM_PROFILE_EN,
        ITEM_PREHEAT_TEMP,
        ITEM_PREHEAT_TIME,
        ITEM_SOAK_TEMP,
        ITEM_SOAK_TIME,
        ITEM_PEAK_TEMP,
        ITEM_TAL_TIME,

        ITEM_HOLD_TIME_LEGACY15S,

        // PID at the very end:
        ITEM_KP,
        ITEM_KI,
        ITEM_KD,

        ITEM_COUNT
    } SettingItem;

    // ===== Screen navigation =====
    if (g_uiScreen == UI_SCREEN_STATUS)
    {
        if (p1) {
            if (Control_IsRunning()) Control_SetRunning(0);
            else                     Control_SetRunning(1);
        }

        if (p2) g_uiScreen = UI_SCREEN_GRAPH;

        if (p3) {
            g_uiScreen   = UI_SCREEN_TUNE;
            g_uiEditMode = 0;
            g_uiSelected = 0;
        }
    }
    else if (g_uiScreen == UI_SCREEN_GRAPH)
    {
        if (p2) g_uiScreen = UI_SCREEN_STATUS;
    }
    else if (g_uiScreen == UI_SCREEN_TUNE)
    {
        // Exit settings: hold B1+B2 => save
        if (h1 && h2) {
            Storage_Save();
            g_uiScreen   = UI_SCREEN_STATUS;
            g_uiEditMode = 0;
            p1 = 0; p2 = 0; p3 = 0;
        }

        if (!g_uiEditMode)
        {
            if (p1) { if (g_uiSelected == 0) g_uiSelected = (uint8_t)(ITEM_COUNT - 1); else g_uiSelected--; }
            if (p2) { g_uiSelected = (uint8_t)((g_uiSelected + 1) % ITEM_COUNT); }
            if (p3) { g_uiEditMode = 1; }
        }
        else
        {
            switch ((SettingItem)g_uiSelected)
            {
                case ITEM_SETPOINT:
                    if (p1) { float v = Control_GetSetpointC(); v += 5.0f; if (v > maxT - 10) v = maxT - 10; if (v < 20) v = 20; Control_SetSetpointC(v); }
                    if (p2) { float v = Control_GetSetpointC(); v -= 5.0f; if (v < 20) v = 20; Control_SetSetpointC(v); }
                    break;

                case ITEM_MAXTEMP:
                    if (p1) { maxT += 5; if (maxT > 320) maxT = 320; Control_SetMaxTempCutoff(maxT); }
                    if (p2) { maxT -= 5; if (maxT < 150) maxT = 150; Control_SetMaxTempCutoff(maxT); }
                    break;

                case ITEM_PROFILE_EN:
                    if (p1 || p2) { profEn = profEn ? 0 : 1; Control_SetProfileEnabled(profEn); }
                    break;

                case ITEM_PREHEAT_TEMP:
                    if (p1) { preT += 5.0f; if (preT > 200.0f) preT = 200.0f; if (preT > maxT - 10) preT = maxT - 10; Control_SetPreheatTempC(preT); }
                    if (p2) { preT -= 5.0f; if (preT < 50.0f)  preT = 50.0f;  Control_SetPreheatTempC(preT); }
                    break;

                case ITEM_PREHEAT_TIME:
                    if (p1) { if (pre15s < 240) pre15s++; Control_SetPreheatTime15s(pre15s); }
                    if (p2) { if (pre15s > 0)   pre15s--; Control_SetPreheatTime15s(pre15s); }
                    break;

                case ITEM_SOAK_TEMP:
                    if (p1) { soakT += 5.0f; if (soakT > 220.0f) soakT = 220.0f; if (soakT > maxT - 10) soakT = maxT - 10; Control_SetSoakTempC(soakT); }
                    if (p2) { soakT -= 5.0f; if (soakT < 80.0f)  soakT = 80.0f;  Control_SetSoakTempC(soakT); }
                    break;

                case ITEM_SOAK_TIME:
                    if (p1) { if (soak15s < 240) soak15s++; Control_SetSoakTime15s(soak15s); }
                    if (p2) { if (soak15s > 0)   soak15s--; Control_SetSoakTime15s(soak15s); }
                    break;

                case ITEM_PEAK_TEMP:
                    if (p1) { peakT += 5.0f; if (peakT > 260.0f) peakT = 260.0f; if (peakT > maxT - 5) peakT = maxT - 5; Control_SetReflowPeakTempC(peakT); }
                    if (p2) { peakT -= 5.0f; if (peakT < 200.0f) peakT = 200.0f; Control_SetReflowPeakTempC(peakT); }
                    break;

                case ITEM_TAL_TIME:
                    if (p1) { if (tal15s < 240) tal15s++; Control_SetTalTime15s(tal15s); }
                    if (p2) { if (tal15s > 0)   tal15s--; Control_SetTalTime15s(tal15s); }
                    break;

                case ITEM_HOLD_TIME_LEGACY15S:
                    if (p1) { if (hold15s < 240) hold15s++; Control_SetHoldTime15s(hold15s); }
                    if (p2) { if (hold15s > 0)   hold15s--; Control_SetHoldTime15s(hold15s); }
                    break;

                // PID moved to the end (still editable/saved)
                case ITEM_KP:
                    if (p1) { kp += 0.5f; if (kp > 20) kp = 20; Control_SetPID(kp, ki, kd); }
                    if (p2) { kp -= 0.5f; if (kp < 0)  kp = 0;  Control_SetPID(kp, ki, kd); }
                    break;

                case ITEM_KI:
                    if (p1) { ki += 0.01f; if (ki > 1.0f) ki = 1.0f; Control_SetPID(kp, ki, kd); }
                    if (p2) { ki -= 0.01f; if (ki < 0)    ki = 0;    Control_SetPID(kp, ki, kd); }
                    break;

                case ITEM_KD:
                    if (p1) { kd += 0.5f; if (kd > 50) kd = 50; Control_SetPID(kp, ki, kd); }
                    if (p2) { kd -= 0.5f; if (kd < 0)  kd = 0;  Control_SetPID(kp, ki, kd); }
                    break;

                default:
                    break;
            }

            // B3 exits edit mode and saves
            if (p3) { g_uiEditMode = 0; Storage_Save(); }
        }
    }

    // ===== Drawing =====
    SSD1306_Fill(0);
    char line[32];
    int  len;

    if (g_uiScreen == UI_SCREEN_STATUS)
    {
        const int FONT_H     = 8;
        const int BAR_H      = 5;

        int START_Y = 2;
        int X_LABEL = 20;
        int X_BAR   = 52;
        int W_BAR   = 54;
        int BAR_OFFSET_Y = -3;

        int y = START_Y;
        int xCenter;

        // Line 1: STATUS (RunState)
        RunState st = Control_GetRunState();
        if (st == RUN_STATE_HEATING) {
            snprintf(line, sizeof(line), "STATUS: HEATING");
        } else if (st == RUN_STATE_HOLDING) {
            uint32_t rem = Control_GetRemainingHoldTimeSeconds();
            snprintf(line, sizeof(line), "HOLD: %02lu:%02lu", (unsigned long)(rem / 60), (unsigned long)(rem % 60));
        } else if (st == RUN_STATE_COOLING) {
            snprintf(line, sizeof(line), "STATUS: COOLING");
        } else if (st == RUN_STATE_DONE) {
            snprintf(line, sizeof(line), "STATUS: DONE");
        } else {
            snprintf(line, sizeof(line), "STATUS: IDLE");
        }
        len = (int)strlen(line);
        xCenter = (128 - (len * 6)) / 2;
        SSD1306_SetCursor((uint8_t)xCenter, (uint8_t)y);
        SSD1306_WriteString(line);
        y += FONT_H;

        // Line 2: PHASE (profile)
        if (Control_GetProfileEnabled()) {
            const char *phs = PhaseToStr(Control_GetProfilePhase());
            snprintf(line, sizeof(line), "PHASE: %s", phs);
        } else {
            snprintf(line, sizeof(line), "PHASE: LEGACY");
        }
        len = (int)strlen(line);
        xCenter = (128 - (len * 6)) / 2;
        SSD1306_SetCursor((uint8_t)xCenter, (uint8_t)y);
        SSD1306_WriteString(line);
        y += FONT_H;

        // Line 3: Current Temp / Errors
        if (errSens)      snprintf(line, sizeof(line), "SENSOR ERROR");
        else if (errOver) snprintf(line, sizeof(line), "OVERTEMP ERROR");
        else              snprintf(line, sizeof(line), "Current: %.1f C", tempF);

        len = (int)strlen(line);
        xCenter = (128 - (len * 6)) / 2;
        SSD1306_SetCursor((uint8_t)xCenter, (uint8_t)y);
        SSD1306_WriteString(line);
        y += FONT_H;

        // Line 4: Target (active)
        snprintf(line, sizeof(line), "Target:  %.0f C", activeSpC);
        len = (int)strlen(line);
        xCenter = (128 - (len * 6)) / 2;
        SSD1306_SetCursor((uint8_t)xCenter, (uint8_t)y);
        SSD1306_WriteString(line);
        y += FONT_H;

        // Line 5: Heat bar
        SSD1306_SetCursor((uint8_t)X_LABEL, (uint8_t)y);
        SSD1306_WriteString("Heat:");
        DrawProgressBar((uint8_t)X_BAR, (uint8_t)(y + BAR_OFFSET_Y), (uint8_t)W_BAR, (uint8_t)BAR_H, (uint8_t)heaterP);
        y += FONT_H;

        // Line 6: Fan bar
        SSD1306_SetCursor((uint8_t)X_LABEL, (uint8_t)y);
        SSD1306_WriteString(" Fan:");
        DrawProgressBar((uint8_t)X_BAR, (uint8_t)(y + BAR_OFFSET_Y), (uint8_t)W_BAR, (uint8_t)BAR_H, fanPct);
    }
    else if (g_uiScreen == UI_SCREEN_TUNE)
    {
        // Header
        SSD1306_SetCursor(40, 0);
        SSD1306_WriteString("SETTINGS");
        for (int i = 0; i < 128; i++) SSD1306_DrawPixel(i, 10, 1);

        // Scroll window so selected item stays visible in the 5-row area
        uint8_t first = 0;
        if (g_uiSelected >= SETTINGS_VISIBLE_ROWS) {
            first = (uint8_t)(g_uiSelected - (SETTINGS_VISIBLE_ROWS - 1));
        }
        if (first > (uint8_t)(ITEM_COUNT - SETTINGS_VISIBLE_ROWS)) {
            first = (uint8_t)(ITEM_COUNT - SETTINGS_VISIBLE_ROWS);
        }

        // Render rows ONLY in the allowed area (y=16..55)
        for (int row = 0; row < SETTINGS_VISIBLE_ROWS; row++)
        {
            uint8_t i = (uint8_t)(first + row);
            int y = SETTINGS_START_Y + (row * SETTINGS_ROW_H);
            if (i >= ITEM_COUNT) break;

            SSD1306_SetCursor(0, (uint8_t)y);
            SSD1306_WriteString((i == g_uiSelected) ? ">" : " ");

            char valStr[16];
            const char *label = "";

            switch ((SettingItem)i)
            {
                case ITEM_SETPOINT:      label = "Setpoint"; snprintf(valStr, sizeof(valStr), "%.0fC", Control_GetSetpointC()); break;
                case ITEM_MAXTEMP:       label = "Safety";   snprintf(valStr, sizeof(valStr), "%.0fC", maxT); break;

                case ITEM_PROFILE_EN:    label = "Profile";  snprintf(valStr, sizeof(valStr), "%s", profEn ? "ON" : "OFF"); break;

                case ITEM_PREHEAT_TEMP:  label = "PreT";     snprintf(valStr, sizeof(valStr), "%.0fC", preT); break;
                case ITEM_PREHEAT_TIME:  label = "PreTime";  FormatMmSsFrom15s(valStr, sizeof(valStr), pre15s); break;

                case ITEM_SOAK_TEMP:     label = "SoakT";    snprintf(valStr, sizeof(valStr), "%.0fC", soakT); break;
                case ITEM_SOAK_TIME:     label = "SoakTim";  FormatMmSsFrom15s(valStr, sizeof(valStr), soak15s); break;

                case ITEM_PEAK_TEMP:     label = "PeakT";    snprintf(valStr, sizeof(valStr), "%.0fC", peakT); break;
                case ITEM_TAL_TIME:      label = "TAL";      FormatMmSsFrom15s(valStr, sizeof(valStr), tal15s); break;

                case ITEM_HOLD_TIME_LEGACY15S:
                    label = "Hold15s";
                    FormatMmSsFrom15s(valStr, sizeof(valStr), hold15s);
                    break;

                // PID last
                case ITEM_KP:            label = "Kp";       snprintf(valStr, sizeof(valStr), "%.1f", kp); break;
                case ITEM_KI:            label = "Ki";       snprintf(valStr, sizeof(valStr), "%.2f", ki); break;
                case ITEM_KD:            label = "Kd";       snprintf(valStr, sizeof(valStr), "%.1f", kd); break;

                default:
                    label = "???";
                    snprintf(valStr, sizeof(valStr), "-");
                    break;
            }

            SSD1306_SetCursor(10, (uint8_t)y);
            if (i == g_uiSelected && g_uiEditMode) {
                snprintf(line, sizeof(line), "%-7s<%6s>", label, valStr);
            } else {
                snprintf(line, sizeof(line), "%-7s %6s ", label, valStr);
            }
            SSD1306_WriteString(line);
        }

        // Legend is always at bottom and never overlaps list
        SSD1306_SetCursor(0, SETTINGS_LEGEND_Y);
        SSD1306_WriteString("B1/B2 Nav  B3 Edit");
    }
    else if (g_uiScreen == UI_SCREEN_GRAPH)
    {
        float minVal = 400.0f, maxVal = -100.0f;
        for (int i = 0; i < GRAPH_WIDTH; i++) {
            if (g_tempHistory[i] < minVal) minVal = g_tempHistory[i];
            if (g_tempHistory[i] > maxVal) maxVal = g_tempHistory[i];
        }

        if (activeSpC < minVal) minVal = activeSpC;
        if (activeSpC > maxVal) maxVal = activeSpC;

        if ((maxVal - minVal) < 10.0f) {
            float mid = (maxVal + minVal) / 2.0f;
            maxVal = mid + 5.0f;
            minVal = mid - 5.0f;
        }
        float range = maxVal - minVal;

        SSD1306_SetCursor(0, 0);
        snprintf(line, sizeof(line), "%.0f", maxVal);
        SSD1306_WriteString(line);

        SSD1306_SetCursor(100, 0);
        snprintf(line, sizeof(line), "%.0f", minVal);
        SSD1306_WriteString(line);

        SSD1306_SetCursor(40, 0);
        snprintf(line, sizeof(line), "Now:%.0f", tempF);
        SSD1306_WriteString(line);

        int spY = GRAPH_HEIGHT + GRAPH_Y_OFFSET - 1
                - (int)((activeSpC - minVal) / range * (float)(GRAPH_HEIGHT - 1));
        if (spY >= GRAPH_Y_OFFSET && spY < 64) {
            for (int i = 0; i < 128; i += 4) {
                SSD1306_DrawPixel(i,   spY, 1);
                SSD1306_DrawPixel(i+1, spY, 1);
            }
        }

        int oldX = 0, oldY = 0;
        int idx  = g_graphWriteIndex;

        for (int x = 0; x < GRAPH_WIDTH; x++)
        {
            float val = g_tempHistory[idx];
            int y = GRAPH_HEIGHT + GRAPH_Y_OFFSET - 1
                  - (int)((val - minVal) / range * (float)(GRAPH_HEIGHT - 1));

            if (y < GRAPH_Y_OFFSET) y = GRAPH_Y_OFFSET;
            if (y > 63)            y = 63;

            if (x == 0) SSD1306_DrawPixel(x, y, 1);
            else        SSD1306_DrawLine(oldX, oldY, x, y, 1);

            oldX = x; oldY = y;
            idx++; if (idx >= GRAPH_WIDTH) idx = 0;
        }
    }

    SSD1306_Update();
}
