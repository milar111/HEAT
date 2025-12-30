/*
 * storage.c
 *
 * Created on: Dec 6, 2025
 * Author: dyord
 */

#include "storage.h"
#include "control.h"
#include "stm32f1xx_hal.h"
#include <string.h>

// Address of the last page of 64KB Flash (Page 63)
// 0x08000000 + 63 * 1024 = 0x0800FC00
#define FLASH_STORAGE_ADDR  0x0800FC00

// Versioning:
// V2 (old): 0xCAFE0002  -> had holdTimeMins (uint8_t)
// V4 (new): 0xCAFE0004  -> has holdTime15s + full profile params
#define STORAGE_MAGIC_V2   0xCAFE0002
#define STORAGE_MAGIC_V4   0xCAFE0004

typedef struct {
    uint32_t magic;
    float    setpoint;
    float    kp;
    float    ki;
    float    kd;
    float    maxTemp;
    uint8_t  holdTimeMins;
    uint8_t  padding[3];
} SettingsDataV2;

typedef struct {
    uint32_t magic;
    float    setpoint;
    float    kp;
    float    ki;
    float    kd;
    float    maxTemp;

    uint16_t holdTime15s;
    uint8_t  profileEnabled;
    uint8_t  padding0;

    float    preheatTempC;
    uint16_t preheatTime15s;
    uint16_t padding1;

    float    soakTempC;
    uint16_t soakTime15s;
    uint16_t padding2;

    float    reflowPeakC;
    uint16_t talTime15s;
    uint16_t padding3;
} SettingsDataV4;

void Storage_Init(void)
{
    uint32_t magic = *(uint32_t *)FLASH_STORAGE_ADDR;

    if (magic == STORAGE_MAGIC_V4)
    {
        SettingsDataV4 *data = (SettingsDataV4 *)FLASH_STORAGE_ADDR;

        Control_SetSetpointC(data->setpoint);
        Control_SetPID(data->kp, data->ki, data->kd);
        Control_SetMaxTempCutoff(data->maxTemp);

        Control_SetHoldTime15s(data->holdTime15s);

        Control_SetProfileEnabled(data->profileEnabled);

        Control_SetPreheatTempC(data->preheatTempC);
        Control_SetPreheatTime15s(data->preheatTime15s);

        Control_SetSoakTempC(data->soakTempC);
        Control_SetSoakTime15s(data->soakTime15s);

        Control_SetReflowPeakTempC(data->reflowPeakC);
        Control_SetTalTime15s(data->talTime15s);

        return;
    }
    else if (magic == STORAGE_MAGIC_V2)
    {
        SettingsDataV2 *data = (SettingsDataV2 *)FLASH_STORAGE_ADDR;

        Control_SetSetpointC(data->setpoint);
        Control_SetPID(data->kp, data->ki, data->kd);
        Control_SetMaxTempCutoff(data->maxTemp);

        Control_SetHoldTime15s((uint16_t)data->holdTimeMins * 4u);

        // migrate with profile OFF
        Control_SetProfileEnabled(0);

        return;
    }

    // Else: Flash empty/unknown => keep defaults set by Control_Init()
}

void Storage_Save(void)
{
    SettingsDataV4 newData;
    memset(&newData, 0, sizeof(newData));

    newData.magic   = STORAGE_MAGIC_V4;

    newData.setpoint = Control_GetSetpointC();
    Control_GetPID(&newData.kp, &newData.ki, &newData.kd);
    newData.maxTemp  = Control_GetMaxTempCutoff();

    newData.holdTime15s    = Control_GetHoldTime15s();
    newData.profileEnabled = Control_GetProfileEnabled();

    newData.preheatTempC   = Control_GetPreheatTempC();
    newData.preheatTime15s = Control_GetPreheatTime15s();

    newData.soakTempC      = Control_GetSoakTempC();
    newData.soakTime15s    = Control_GetSoakTime15s();

    newData.reflowPeakC    = Control_GetReflowPeakTempC();
    newData.talTime15s     = Control_GetTalTime15s();

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;

    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_STORAGE_ADDR;
    EraseInitStruct.NbPages     = 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) == HAL_OK)
    {
        uint32_t *source = (uint32_t *)&newData;
        uint32_t address = FLASH_STORAGE_ADDR;

        for (int i = 0; i < (int)(sizeof(SettingsDataV4) / 4); i++)
        {
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, source[i]);
            address += 4;
        }
    }

    HAL_FLASH_Lock();
}
