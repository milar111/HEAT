#ifndef SSD1306_H
#define SSD1306_H

#include "stm32f1xx_hal.h"

void SSD1306_Init(I2C_HandleTypeDef *hi2c);
void SSD1306_Fill(uint8_t color); // 0 = Black, 1 = White
void SSD1306_Update(void);

void SSD1306_SetCursor(uint8_t x, uint8_t y);
void SSD1306_WriteChar(char c);
void SSD1306_WriteString(const char *str);

// New Graphic Functions
void SSD1306_DrawPixel(int16_t x, int16_t y, uint8_t color);
void SSD1306_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t color);

#endif
