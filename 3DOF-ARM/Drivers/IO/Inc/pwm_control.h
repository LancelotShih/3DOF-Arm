/* ABOUTME: Controls hobby servo angles through STM32 timer PWM channels.
 * ABOUTME: Starts configured channels, maps degrees to pulse widths, and resets servos to zero.
 */
#ifndef PWM_CONTROL_H
#define PWM_CONTROL_H

#include "stm32f4xx_hal.h"

#define PWM_CONTROL_MIN_ANGLE_DEGREES 0U
#define PWM_CONTROL_MAX_ANGLE_DEGREES 180U
#define PWM_CONTROL_RESET_BASE_ANGLE_DEGREES 90U
#define PWM_CONTROL_RESET_HEIGHT_ANGLE_DEGREES 90U
#define PWM_CONTROL_RESET_DEPTH_ANGLE_DEGREES 102U
#define PWM_CONTROL_RESET_CLAW_ANGLE_DEGREES 0U
#define PWM_CONTROL_SLEW_STEP_DEG    1U
#define PWM_CONTROL_SLEW_INTERVAL_MS 20U

HAL_StatusTypeDef PWM_Control_Start(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t reset_angle_degrees);
HAL_StatusTypeDef PWM_Control_SetAngle(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t angle_degrees);
HAL_StatusTypeDef PWM_Control_ResetAngle(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t reset_angle_degrees);
uint16_t PWM_Control_NextSweepAngle(uint16_t current_angle_degrees, int8_t *direction, uint16_t minimum_angle_degrees, uint16_t maximum_angle_degrees, uint16_t step_degrees);
void SetAllServoAngles(uint16_t angle_degrees);
void PWM_Control_SetTarget(uint32_t channel, uint16_t angle_degrees);
void PWM_Control_SlewUpdate(TIM_HandleTypeDef *htim);
uint16_t PWM_Control_GetCurrentAngle(uint32_t channel);

#endif /* PWM_CONTROL_H */
