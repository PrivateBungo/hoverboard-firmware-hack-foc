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

#ifndef VELOCITY_SETPOINT_LAYER_H
#define VELOCITY_SETPOINT_LAYER_H

#include <stdint.h>

typedef struct {
  int16_t velocity_setpoint;
  int16_t acceleration_setpoint;
  uint8_t slip_gap_clamp_active;
  uint16_t slip_gap_release_counter;
} VelocitySetpointLayerState;

typedef struct {
  int16_t velocity_setpoint;
  int16_t acceleration_setpoint;
  uint8_t slip_gap_clamp_active;
} VelocitySetpointLayerOutput;

void VelocitySetpointLayer_Init(VelocitySetpointLayerState *state);
void VelocitySetpointLayer_Reset(VelocitySetpointLayerState *state);
void VelocitySetpointLayer_Update(VelocitySetpointLayerState *state,
                                  int16_t velocity_intent,
                                  int16_t velocity_actual,
                                  int16_t speed_max_rpm,
                                  VelocitySetpointLayerOutput *output);

#endif // VELOCITY_SETPOINT_LAYER_H
