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

void VelocitySetpointLayer_Init(VelocitySetpointLayerState *state) {
  VelocitySetpointLayer_Reset(state);
}

void VelocitySetpointLayer_Reset(VelocitySetpointLayerState *state) {
  if (state == 0) {
    return;
  }

  state->velocity_setpoint = 0;
  state->acceleration_setpoint = 0;
}

void VelocitySetpointLayer_Update(VelocitySetpointLayerState *state,
                                  int16_t velocity_intent,
                                  VelocitySetpointLayerOutput *output) {
  int16_t previous_velocity;

  previous_velocity = state->velocity_setpoint;
  state->velocity_setpoint = velocity_intent;
  state->acceleration_setpoint = (int16_t)(state->velocity_setpoint - previous_velocity);

  output->velocity_setpoint = state->velocity_setpoint;
  output->acceleration_setpoint = state->acceleration_setpoint;
  output->slip_gap_clamp_active = 0U;
}
