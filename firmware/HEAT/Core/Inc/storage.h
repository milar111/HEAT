#ifndef STORAGE_H
#define STORAGE_H

#include <stdint.h>

// Initialize storage: Loads settings from flash into Control module.
// Call this in main.c after Control_Init()
void Storage_Init(void);

// Save current Control settings to flash.
// Call this in ui.c when exiting the settings menu.
void Storage_Save(void);

#endif // STORAGE_H
