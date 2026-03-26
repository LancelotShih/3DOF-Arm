/* ABOUTME: Formats servo angle text and renders it on the SSD1306 status display.
 * ABOUTME: Shows the live arm state without coupling the OLED code to the UART parser.
 */
#include "oled_status.h"

#include "fonts.h"
#include "ssd1306.h"

static void OLED_Status_FormatLine(char *line, const char *label, uint16_t angle)
{
  uint16_t clamped_angle = (angle > 999U) ? 999U : angle;

  line[0] = label[0];
  line[1] = label[1];
  line[2] = label[2];
  line[3] = label[3];
  line[4] = ' ';
  line[5] = (char)('0' + ((clamped_angle / 100U) % 10U));
  line[6] = (char)('0' + ((clamped_angle / 10U) % 10U));
  line[7] = (char)('0' + (clamped_angle % 10U));
  line[8] = '\0';
}

HAL_StatusTypeDef OLED_Status_Init(void)
{
  return SSD1306_Init();
}

HAL_StatusTypeDef OLED_Status_ShowAngles(const uint16_t angles[OLED_STATUS_SERVO_COUNT])
{
  static const char *labels[OLED_STATUS_SERVO_COUNT] = {"Base", "Hght", "Dpth", "Claw"};
  static const uint16_t rows[OLED_STATUS_SERVO_COUNT] = {0U, 16U, 32U, 48U};
  uint32_t index;
  char line[9];

  if (angles == 0)
  {
    return HAL_ERROR;
  }

  SSD1306_Fill(SSD1306_COLOR_BLACK);

  for (index = 0U; index < OLED_STATUS_SERVO_COUNT; index++)
  {
    OLED_Status_FormatLine(line, labels[index], angles[index]);
    if (SSD1306_Puts(0U, rows[index], line, &Font_7x10) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  return SSD1306_UpdateScreen();
}
