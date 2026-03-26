/* ABOUTME: Declares bitmap fonts used by the SSD1306 text renderer.
 * ABOUTME: Exposes fixed-width ASCII font tables for the OLED status display.
 */
#ifndef FONTS_H
#define FONTS_H

#include "stm32f4xx_hal.h"

typedef struct {
  uint8_t FontWidth;
  uint8_t FontHeight;
  const uint16_t *data;
} FontDef_t;

extern FontDef_t Font_7x10;
extern FontDef_t Font_11x18;

#endif /* FONTS_H */
