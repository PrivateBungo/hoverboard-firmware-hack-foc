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

static int16_t VelocitySetpointLayer_ComputeStep(int16_t target,
                                                 int16_t current,
                                                 uint16_t stepUp,
                                                 uint16_t stepDown) {
  int32_t delta = (int32_t)target - current;

  if (delta > 0) {
    int32_t step = (stepUp > 0U) ? (int32_t)stepUp : 1;
    if (delta > step) {
      return (int16_t)(current + step);
    }
    return target;
  }

  if (delta < 0) {
    int32_t step = (stepDown > 0U) ? (int32_t)stepDown : 1;
    if (-delta > step) {
      return (int16_t)(current - step);
    }
    return target;
  }

  return current;
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
                                  VelocitySetpointLayerOutput *output) {
  int16_t previous_velocity;
  int16_t target_velocity;
  int16_t shaped_velocity;
  int16_t slip_gap;
  int16_t slip_gap_abs;
  int16_t slip_gap_release_max;
  uint8_t slip_gap_clamped;

  if ((state == 0) || (output == 0)) {
    return;
  }

  target_velocity = VelocitySetpointLayer_ClampS16(velocity_intent,
                                                    VELOCITY_SETPOINT_CMD_MIN,
                                                    VELOCITY_SETPOINT_CMD_MAX);

  previous_velocity = state->velocity_setpoint;
  shaped_velocity = VelocitySetpointLayer_ComputeStep(target_velocity,
                                                      previous_velocity,
                                                      SETPOINT_RATE_UP,
                                                      SETPOINT_RATE_DOWN);

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
  }

  state->slip_gap_clamp_active = slip_gap_clamped;

  state->velocity_setpoint = shaped_velocity;
  state->acceleration_setpoint = (int16_t)(state->velocity_setpoint - previous_velocity);

  output->velocity_setpoint = state->velocity_setpoint;
  output->acceleration_setpoint = state->acceleration_setpoint;
  output->slip_gap_clamp_active = state->slip_gap_clamp_active;
}
