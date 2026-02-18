/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2026 OpenAI
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*/

#include "motor_controller.h"
#include "config.h"

#define MOTOR_CONTROLLER_CMD_MAX  (1000)
#define MOTOR_CONTROLLER_CMD_MIN  (-MOTOR_CONTROLLER_CMD_MAX)

static int16_t MotorController_ClampS16(int32_t value, int16_t lower, int16_t upper) {
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return (int16_t)value;
}

void MotorController_Init(MotorControllerState *state) {
  MotorController_Reset(state);
}

void MotorController_Reset(MotorControllerState *state) {
  if (state == 0) {
    return;
  }

  state->integrator_q15 = 0;
  state->torque_cmd = 0;
}

void MotorController_Update(MotorControllerState *state,
                            int16_t velocity_setpoint_cmd,
                            int16_t speed_measured_rpm,
                            int16_t speed_max_rpm,
                            uint8_t control_enabled,
                            MotorControllerOutput *output) {
  int32_t speed_ref_rpm;
  int32_t speed_err_rpm;
  int32_t p_term;
  int32_t i_term;
  int32_t i_delta;
  int32_t raw_cmd;
  int16_t cmd_limited;
  int16_t torque_limit;
  uint8_t saturating;

  if ((state == 0) || (output == 0)) {
    return;
  }

  if ((control_enabled == 0U) || (speed_max_rpm <= 0)) {
    MotorController_Reset(state);
    output->torque_cmd = 0;
    output->speed_error_rpm = 0;
    output->saturated = 0U;
    return;
  }

  speed_ref_rpm = ((int32_t)velocity_setpoint_cmd * speed_max_rpm) / MOTOR_CONTROLLER_CMD_MAX;
  speed_err_rpm = speed_ref_rpm - speed_measured_rpm;

  p_term = (int32_t)((((int64_t)speed_err_rpm * MOTOR_CTRL_VEL_KP_Q15) + 16384LL) >> 15);

  i_delta = (int32_t)((((int64_t)speed_err_rpm * MOTOR_CTRL_VEL_KI_Q15) + 16384LL) >> 15);
  state->integrator_q15 += i_delta;
  state->integrator_q15 = MotorController_ClampS16(state->integrator_q15,
                                                   (int16_t)-MOTOR_CTRL_INT_LIM,
                                                   (int16_t)MOTOR_CTRL_INT_LIM);

  i_term = state->integrator_q15;
  raw_cmd = p_term + i_term;

  torque_limit = (MOTOR_CTRL_TORQUE_MAX > 0) ? MOTOR_CTRL_TORQUE_MAX : MOTOR_CONTROLLER_CMD_MAX;
  if (torque_limit > MOTOR_CONTROLLER_CMD_MAX) {
    torque_limit = MOTOR_CONTROLLER_CMD_MAX;
  }

  cmd_limited = MotorController_ClampS16(raw_cmd, (int16_t)-torque_limit, torque_limit);
  saturating = (uint8_t)(cmd_limited != (int16_t)raw_cmd);

  if (saturating != 0U) {
    uint8_t saturation_same_error_sign = (uint8_t)(((cmd_limited > 0) && (speed_err_rpm > 0)) ||
                                                   ((cmd_limited < 0) && (speed_err_rpm < 0)));
    if (saturation_same_error_sign != 0U) {
      state->integrator_q15 -= i_delta;
      i_term = state->integrator_q15;
      raw_cmd = p_term + i_term;
      cmd_limited = MotorController_ClampS16(raw_cmd, (int16_t)-torque_limit, torque_limit);
      saturating = (uint8_t)(cmd_limited != (int16_t)raw_cmd);
    }
  }

  state->torque_cmd = cmd_limited;

  output->torque_cmd = state->torque_cmd;
  output->speed_error_rpm = MotorController_ClampS16(speed_err_rpm, -32767, 32767);
  output->saturated = saturating;
}
