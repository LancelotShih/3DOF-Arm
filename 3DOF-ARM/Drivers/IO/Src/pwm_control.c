/* ABOUTME: Implements servo angle commands on top of STM32 timer compare channels.
 * ABOUTME: Uses a 50 Hz PWM frame with 1 ms to 2 ms pulses for 0 to 180 degree commands.
 */
#include "pwm_control.h"
#include "main.h"

extern TIM_HandleTypeDef htim3;

#define PWM_CONTROL_MIN_PULSE_TICKS 1000U
#define PWM_CONTROL_MAX_PULSE_TICKS 2000U

void SetAllServoAngles(uint16_t angle_degrees)
{
  if ((PWM_Control_SetAngle(&htim3, TIM_CHANNEL_1, angle_degrees) != HAL_OK) ||
      (PWM_Control_SetAngle(&htim3, TIM_CHANNEL_2, angle_degrees) != HAL_OK) ||
      (PWM_Control_SetAngle(&htim3, TIM_CHANNEL_3, angle_degrees) != HAL_OK) ||
      (PWM_Control_SetAngle(&htim3, TIM_CHANNEL_4, angle_degrees) != HAL_OK))
  {
    Error_Handler();
  }
}

static uint8_t PWM_Control_ChannelIsSupported(uint32_t channel)
{
  switch (channel)
  {
    case TIM_CHANNEL_1:
    case TIM_CHANNEL_2:
    case TIM_CHANNEL_3:
    case TIM_CHANNEL_4:
      return 1U;
    default:
      return 0U;
  }
}

static uint16_t PWM_Control_ClampAngle(uint16_t angle_degrees)
{
  if (angle_degrees > PWM_CONTROL_MAX_ANGLE_DEGREES)
  {
    return PWM_CONTROL_MAX_ANGLE_DEGREES;
  }

  return angle_degrees;
}

static uint32_t PWM_Control_CompareFromAngle(uint16_t angle_degrees)
{
  uint32_t clamped_angle = PWM_Control_ClampAngle(angle_degrees);
  uint32_t pulse_span = PWM_CONTROL_MAX_PULSE_TICKS - PWM_CONTROL_MIN_PULSE_TICKS;

  return PWM_CONTROL_MIN_PULSE_TICKS + ((pulse_span * clamped_angle) / PWM_CONTROL_MAX_ANGLE_DEGREES);
}

HAL_StatusTypeDef PWM_Control_Start(TIM_HandleTypeDef *htim, uint32_t channel)
{
  HAL_StatusTypeDef status;

  if ((htim == 0) || (PWM_Control_ChannelIsSupported(channel) == 0U))
  {
    return HAL_ERROR;
  }

  __HAL_TIM_SET_COMPARE(htim, channel, PWM_Control_CompareFromAngle(PWM_CONTROL_RESET_ANGLE_DEGREES));
  status = HAL_TIM_PWM_Start(htim, channel);

  return status;
}

HAL_StatusTypeDef PWM_Control_SetAngle(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t angle_degrees)
{
  if ((htim == 0) || (PWM_Control_ChannelIsSupported(channel) == 0U))
  {
    return HAL_ERROR;
  }

  __HAL_TIM_SET_COMPARE(htim, channel, PWM_Control_CompareFromAngle(angle_degrees));

  return HAL_OK;
}

HAL_StatusTypeDef PWM_Control_ResetAngle(TIM_HandleTypeDef *htim, uint32_t channel)
{
  return PWM_Control_SetAngle(htim, channel, PWM_CONTROL_RESET_ANGLE_DEGREES);
}

uint16_t PWM_Control_NextSweepAngle(uint16_t current_angle_degrees, int8_t *direction, uint16_t minimum_angle_degrees, uint16_t maximum_angle_degrees, uint16_t step_degrees)
{
  uint16_t clamped_current = current_angle_degrees;

  if ((direction == 0) || (step_degrees == 0U) || (minimum_angle_degrees >= maximum_angle_degrees))
  {
    return current_angle_degrees;
  }

  if (clamped_current < minimum_angle_degrees)
  {
    clamped_current = minimum_angle_degrees;
  }
  else if (clamped_current > maximum_angle_degrees)
  {
    clamped_current = maximum_angle_degrees;
  }

  if (*direction >= 0)
  {
    if (clamped_current >= (maximum_angle_degrees - step_degrees))
    {
      *direction = -1;
      return maximum_angle_degrees;
    }

    return clamped_current + step_degrees;
  }

  if (clamped_current <= (minimum_angle_degrees + step_degrees))
  {
    *direction = 1;
    return minimum_angle_degrees;
  }

  return clamped_current - step_degrees;
}
