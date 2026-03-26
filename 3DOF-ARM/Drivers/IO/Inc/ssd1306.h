/* ABOUTME: Drives a 128x64 SSD1306 OLED over STM32 I2C with a shadow framebuffer.
 * ABOUTME: Provides screen init, pixel drawing, and fixed-width ASCII text output.
 */
#ifndef SSD1306_H
#define SSD1306_H

#include "fonts.h"
#include "stm32f4xx_hal.h"

#define SSD1306_I2C_ADDR 0x78U
#define SSD1306_WIDTH 128U
#define SSD1306_HEIGHT 64U
#define SSD1306_TIMEOUT_MS 100U

typedef enum {
  SSD1306_COLOR_BLACK = 0x00,
  SSD1306_COLOR_WHITE = 0x01
} SSD1306_COLOR_t;

HAL_StatusTypeDef SSD1306_Init(void);
void SSD1306_Fill(SSD1306_COLOR_t color);
HAL_StatusTypeDef SSD1306_UpdateScreen(void);
HAL_StatusTypeDef SSD1306_SetPixel(uint16_t x, uint16_t y, SSD1306_COLOR_t color);
HAL_StatusTypeDef SSD1306_Putc(uint16_t x, uint16_t y, char ch, const FontDef_t *font);
HAL_StatusTypeDef SSD1306_Puts(uint16_t x, uint16_t y, const char *str, const FontDef_t *font);

#endif /* SSD1306_H */
