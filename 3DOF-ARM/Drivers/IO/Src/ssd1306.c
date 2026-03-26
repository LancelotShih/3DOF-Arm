/* ABOUTME: Implements SSD1306 OLED drawing over I2C2 using an in-memory framebuffer.
 * ABOUTME: Supports display init, buffered updates, and ASCII text rendering.
 */
#include "ssd1306.h"

#include <string.h>

extern I2C_HandleTypeDef hi2c2;

static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8U];

static const uint8_t SSD1306_InitSequence[] = {
  0xAE, 0x20, 0x10, 0xB0, 0xC8, 0x00, 0x10, 0x40, 0x81, 0xFF,
  0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3, 0x00, 0xD5, 0xF0, 0xD9,
  0x22, 0xDA, 0x12, 0xDB, 0x20, 0x8D, 0x14, 0xAF, 0x2E
};

static HAL_StatusTypeDef SSD1306_Write(uint8_t control, const uint8_t *data, uint16_t size)
{
  uint8_t packet[SSD1306_WIDTH + 1U];

  if ((data == 0) || (size > SSD1306_WIDTH))
  {
    return HAL_ERROR;
  }

  packet[0] = control;
  memcpy(&packet[1], data, size);
  return HAL_I2C_Master_Transmit(&hi2c2, SSD1306_I2C_ADDR, packet, (uint16_t)(size + 1U), SSD1306_TIMEOUT_MS);
}

HAL_StatusTypeDef SSD1306_Init(void)
{
  if (HAL_I2C_IsDeviceReady(&hi2c2, SSD1306_I2C_ADDR, 1U, SSD1306_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  HAL_Delay(10U);

  if (SSD1306_Write(0x00U, SSD1306_InitSequence, sizeof(SSD1306_InitSequence)) != HAL_OK)
  {
    return HAL_ERROR;
  }

  SSD1306_Fill(SSD1306_COLOR_BLACK);
  return SSD1306_UpdateScreen();
}

void SSD1306_Fill(SSD1306_COLOR_t color)
{
  memset(SSD1306_Buffer, (color == SSD1306_COLOR_BLACK) ? 0x00 : 0xFF, sizeof(SSD1306_Buffer));
}

HAL_StatusTypeDef SSD1306_UpdateScreen(void)
{
  uint8_t page;

  for (page = 0U; page < 8U; page++)
  {
    uint8_t commands[3] = {(uint8_t)(0xB0U + page), 0x00U, 0x10U};

    if (SSD1306_Write(0x00U, commands, sizeof(commands)) != HAL_OK)
    {
      return HAL_ERROR;
    }

    if (SSD1306_Write(0x40U, &SSD1306_Buffer[SSD1306_WIDTH * page], SSD1306_WIDTH) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}

HAL_StatusTypeDef SSD1306_SetPixel(uint16_t x, uint16_t y, SSD1306_COLOR_t color)
{
  if ((x >= SSD1306_WIDTH) || (y >= SSD1306_HEIGHT))
  {
    return HAL_ERROR;
  }

  if (color == SSD1306_COLOR_WHITE)
  {
    SSD1306_Buffer[x + ((y / 8U) * SSD1306_WIDTH)] |= (uint8_t)(1U << (y % 8U));
  }
  else
  {
    SSD1306_Buffer[x + ((y / 8U) * SSD1306_WIDTH)] &= (uint8_t)~(1U << (y % 8U));
  }

  return HAL_OK;
}

HAL_StatusTypeDef SSD1306_Putc(uint16_t x, uint16_t y, char ch, const FontDef_t *font)
{
  uint32_t row;
  uint32_t column;

  if ((font == 0) || (ch < 32) || ((uint16_t)x + font->FontWidth > SSD1306_WIDTH) ||
      ((uint16_t)y + font->FontHeight > SSD1306_HEIGHT))
  {
    return HAL_ERROR;
  }

  for (row = 0U; row < font->FontHeight; row++)
  {
    uint16_t bitmap = font->data[((uint32_t)(ch - 32) * font->FontHeight) + row];

    for (column = 0U; column < font->FontWidth; column++)
    {
      SSD1306_SetPixel((uint16_t)(x + column), (uint16_t)(y + row),
                       (bitmap & (uint16_t)(1U << (15U - column))) ? SSD1306_COLOR_WHITE : SSD1306_COLOR_BLACK);
    }
  }

  return HAL_OK;
}

HAL_StatusTypeDef SSD1306_Puts(uint16_t x, uint16_t y, const char *str, const FontDef_t *font)
{
  uint16_t cursor_x = x;
  uint16_t cursor_y = y;
  uint16_t origin_x = x;

  if ((str == 0) || (font == 0))
  {
    return HAL_ERROR;
  }

  while (*str != '\0')
  {
    if (*str == '\n')
    {
      cursor_x = origin_x;
      cursor_y = (uint16_t)(cursor_y + font->FontHeight);
      str++;
      continue;
    }

    if (SSD1306_Putc(cursor_x, cursor_y, *str, font) != HAL_OK)
    {
      return HAL_ERROR;
    }

    cursor_x = (uint16_t)(cursor_x + font->FontWidth);
    str++;
  }

  return HAL_OK;
}
