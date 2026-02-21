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

#include "velocity_setpoint_layer.h"
#include "config.h"

#define VELOCITY_SETPOINT_CMD_MAX  (1000)
#define VELOCITY_SETPOINT_CMD_MIN  (-VELOCITY_SETPOINT_CMD_MAX)

static int16_t VelocitySetpointLayer_ClampS16(int32_t value, int16_t lower, int16_t upper) {
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return (int16_t)value;
}

static int16_t VelocitySetpointLayer_ClampDeltaToAccelLimit(int16_t velocity_delta,
                                                             int16_t accel_up_cmd_per_loop,
                                                             int16_t accel_down_cmd_per_loop) {
  if (velocity_delta >= 0) {
    return VelocitySetpointLayer_ClampS16(velocity_delta, 0, accel_up_cmd_per_loop);
  }

  return VelocitySetpointLayer_ClampS16(velocity_delta,
                                        (int16_t)-accel_down_cmd_per_loop,
                                        0);
}

static int16_t VelocitySetpointLayer_ConvertAccelMmps2ToCmdPerLoop(int32_t accel_mmps2,
                                                                    int16_t speed_max_rpm) {
  int64_t num;
  int64_t den;
  int32_t cmd_per_loop;

  if ((accel_mmps2 <= 0) || (speed_max_rpm <= 0) || (SETPOINT_WHEEL_DIAMETER_MM <= 0)) {
    return 1;
  }

  /*
   * Conversion chain:
   *   accel_mmps2 -> rpm/s = accel * 60 / circumference_mm
   *   cmd/s = rpm/s * 1000 / speed_max_rpm
   *   cmd/loop = cmd/s * loop_ms / 1000
   * Simplified integer formula:
   *   cmd/loop = accel_mmps2 * 60 * loop_ms / (circumference_mm * speed_max_rpm)
   */
  num = (int64_t)accel_mmps2 * 60LL * (int64_t)DELAY_IN_MAIN_LOOP;
  den = (int64_t)SETPOINT_WHEEL_CIRCUMFERENCE_MM * (int64_t)speed_max_rpm;
  if (den <= 0) {
    return 1;
  }

  cmd_per_loop = (int32_t)((num + (den / 2LL)) / den);
  if (cmd_per_loop < 1) {
    cmd_per_loop = 1;
  }

  return VelocitySetpointLayer_ClampS16(cmd_per_loop, 1, VELOCITY_SETPOINT_CMD_MAX);
}

static int16_t VelocitySetpointLayer_ConvertJerkMmps3ToCmdPerLoop2(int32_t jerk_mmps3,
                                                                    int16_t speed_max_rpm) {
  int64_t num;
  int64_t den;
  int32_t cmd_per_loop2;
  int64_t loop_ms_sq;

  if ((jerk_mmps3 <= 0) || (speed_max_rpm <= 0) || (SETPOINT_WHEEL_DIAMETER_MM <= 0)) {
    return 1;
  }

  /*
   * Conversion chain:
   *   jerk_mmps3 -> rpm/s^2 = jerk * 60 / circumference_mm
   *   cmd/s^2 = rpm/s^2 * 1000 / speed_max_rpm
   *   cmd/loop^2 = cmd/s^2 * (loop_ms/1000)^2
   * Simplified integer formula:
   *   cmd/loop^2 = jerk_mmps3 * 60 * loop_ms^2 / (circumference_mm * speed_max_rpm * 1000)
   */
  loop_ms_sq = (int64_t)DELAY_IN_MAIN_LOOP * (int64_t)DELAY_IN_MAIN_LOOP;
  num = (int64_t)jerk_mmps3 * 60LL * loop_ms_sq;
  den = ((int64_t)SETPOINT_WHEEL_CIRCUMFERENCE_MM * (int64_t)speed_max_rpm * 1000LL);
  if (den <= 0) {
    return 1;
  }

  cmd_per_loop2 = (int32_t)((num + (den / 2LL)) / den);
  if (cmd_per_loop2 < 1) {
    cmd_per_loop2 = 1;
  }

  return VelocitySetpointLayer_ClampS16(cmd_per_loop2, 1, VELOCITY_SETPOINT_CMD_MAX);
}

static int16_t VelocitySetpointLayer_ComputeJerkLimitedAccel(int16_t accel_current,
                                                              int16_t accel_target,
                                                              int16_t jerk_up_cmd_per_loop2,
                                                              int16_t jerk_down_cmd_per_loop2) {
  int32_t delta = (int32_t)accel_target - accel_current;
  int16_t jerk_limit = (accel_target >= 0) ? jerk_up_cmd_per_loop2 : jerk_down_cmd_per_loop2;

  if (delta > jerk_limit) {
    return (int16_t)(accel_current + jerk_limit);
  }

  if (delta < -jerk_limit) {
    return (int16_t)(accel_current - jerk_limit);
  }

  return accel_target;
}

void VelocitySetpointLayer_Init(VelocitySetpointLayerState *state) {
  VelocitySetpointLayer_Reset(state);
}

void VelocitySetpointLayer_Reset(VelocitySetpointLayerState *state) {
  if (state == 0) {
    return;
  }

  state->velocity_setpoint = 0;
  state->acceleration_setpoint = 0;
  state->slip_gap_clamp_active = 0U;
  state->slip_gap_release_counter = 0U;
}

void VelocitySetpointLayer_Update(VelocitySetpointLayerState *state,
                                  int16_t velocity_intent,
                                  int16_t velocity_actual,
                                  int16_t speed_max_rpm,
                                  VelocitySetpointLayerOutput *output) {
  int16_t previous_velocity;
  int16_t target_velocity;
  int16_t shaped_velocity;
  int16_t accel_target;
  int16_t accel_next;
  int16_t accel_up_cmd_per_loop;
  int16_t accel_down_cmd_per_loop;
  int16_t jerk_up_cmd_per_loop2;
  int16_t jerk_down_cmd_per_loop2;
  int16_t velocity_delta;
  int16_t slip_gap;
  int16_t slip_gap_abs;
  int16_t slip_gap_release_max;
  uint8_t slip_gap_clamped;

  if ((state == 0) || (output == 0)) {
    return;
  }

  if (speed_max_rpm <= 0) {
    speed_max_rpm = SETPOINT_SPEED_MAX_RPM_FALLBACK;
  }

  target_velocity = VelocitySetpointLayer_ClampS16(velocity_intent,
                                                    VELOCITY_SETPOINT_CMD_MIN,
                                                    VELOCITY_SETPOINT_CMD_MAX);

  accel_up_cmd_per_loop = VelocitySetpointLayer_ConvertAccelMmps2ToCmdPerLoop(SETPOINT_ACCEL_UP_MMPS2,
                                                                               speed_max_rpm);
  accel_down_cmd_per_loop = VelocitySetpointLayer_ConvertAccelMmps2ToCmdPerLoop(SETPOINT_ACCEL_DOWN_MMPS2,
                                                                                 speed_max_rpm);
  jerk_up_cmd_per_loop2 = VelocitySetpointLayer_ConvertJerkMmps3ToCmdPerLoop2(SETPOINT_JERK_UP_MMPS3,
                                                                               speed_max_rpm);
  jerk_down_cmd_per_loop2 = VelocitySetpointLayer_ConvertJerkMmps3ToCmdPerLoop2(SETPOINT_JERK_DOWN_MMPS3,
                                                                                 speed_max_rpm);

  previous_velocity = state->velocity_setpoint;
  velocity_delta = (int16_t)(target_velocity - previous_velocity);
  accel_target = VelocitySetpointLayer_ClampDeltaToAccelLimit(velocity_delta,
                                                               accel_up_cmd_per_loop,
                                                               accel_down_cmd_per_loop);
  accel_next = VelocitySetpointLayer_ComputeJerkLimitedAccel(state->acceleration_setpoint,
                                                             accel_target,
                                                             jerk_up_cmd_per_loop2,
                                                             jerk_down_cmd_per_loop2);

  shaped_velocity = (int16_t)(previous_velocity + accel_next);
  if (((velocity_delta > 0) && (shaped_velocity > target_velocity)) ||
      ((velocity_delta < 0) && (shaped_velocity < target_velocity))) {
    shaped_velocity = target_velocity;
    accel_next = (int16_t)(shaped_velocity - previous_velocity);
  }

  slip_gap_clamped = state->slip_gap_clamp_active;
  slip_gap = (int16_t)(shaped_velocity - velocity_actual);
  slip_gap_abs = (slip_gap >= 0) ? slip_gap : (int16_t)-slip_gap;

  if (slip_gap_clamped == 0U) {
    if (slip_gap_abs > SETPOINT_SLIP_GAP_ON) {
      slip_gap_clamped = 1U;
      state->slip_gap_release_counter = 0U;
    }
  } else {
    if (slip_gap_abs <= SETPOINT_SLIP_GAP_OFF) {
      uint16_t releaseCounter = (uint16_t)(state->slip_gap_release_counter + 1U);
      state->slip_gap_release_counter = (releaseCounter < SETPOINT_SLIP_RELEASE_LOOPS) ? releaseCounter : SETPOINT_SLIP_RELEASE_LOOPS;
      if (state->slip_gap_release_counter >= SETPOINT_SLIP_RELEASE_LOOPS) {
        slip_gap_clamped = 0U;
        state->slip_gap_release_counter = 0U;
      }
    } else {
      state->slip_gap_release_counter = 0U;
    }
  }

  if (slip_gap_clamped != 0U) {
    slip_gap_release_max = (int16_t)((SETPOINT_SLIP_GAP_ON > SETPOINT_SLIP_GAP_OFF) ? SETPOINT_SLIP_GAP_ON : SETPOINT_SLIP_GAP_OFF);
    if (slip_gap > slip_gap_release_max) {
      shaped_velocity = VelocitySetpointLayer_ClampS16((int32_t)velocity_actual + slip_gap_release_max,
                                                        VELOCITY_SETPOINT_CMD_MIN,
                                                        VELOCITY_SETPOINT_CMD_MAX);
    } else if (slip_gap < -slip_gap_release_max) {
      shaped_velocity = VelocitySetpointLayer_ClampS16((int32_t)velocity_actual - slip_gap_release_max,
                                                        VELOCITY_SETPOINT_CMD_MIN,
                                                        VELOCITY_SETPOINT_CMD_MAX);
    }
    accel_next = (int16_t)(shaped_velocity - previous_velocity);
  }

  state->slip_gap_clamp_active = slip_gap_clamped;
  state->velocity_setpoint = shaped_velocity;
  state->acceleration_setpoint = accel_next;

  output->velocity_setpoint = state->velocity_setpoint;
  output->acceleration_setpoint = state->acceleration_setpoint;
  output->slip_gap_clamp_active = state->slip_gap_clamp_active;
}
