/* ABOUTME: Renders the arm's four servo angles as ASCII text on the OLED screen.
 * ABOUTME: Keeps display formatting separate from the PWM controller and UART parser.
 */
#ifndef OLED_STATUS_H
#define OLED_STATUS_H

#include "stm32f4xx_hal.h"

#define OLED_STATUS_SERVO_COUNT 4U

HAL_StatusTypeDef OLED_Status_Init(void);
HAL_StatusTypeDef OLED_Status_ShowAngles(const uint16_t angles[OLED_STATUS_SERVO_COUNT]);

#endif /* OLED_STATUS_H */
