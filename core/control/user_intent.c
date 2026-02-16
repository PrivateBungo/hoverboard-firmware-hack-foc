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

#include "user_intent.h"

#include "config.h"
#include "drive_math.h"

void UserIntent_Init(UserIntentState *state) {
  UserIntent_Reset(state);
}

void UserIntent_Reset(UserIntentState *state) {
  state->steeringRateFixdt = 0;
  state->longitudinalRateFixdt = 0;
  state->steeringFixdt = 0;
  state->longitudinalFixdt = 0;
}

void UserIntent_BuildLongitudinalSteeringIntent(UserIntentState *state,
                                                int16_t steeringCmd,
                                                int16_t longitudinalCmd,
                                                uint16_t rate,
                                                int16_t *steeringIntent,
                                                int16_t *longitudinalIntent) {
  rateLimiter16(steeringCmd, rate, &state->steeringRateFixdt);
  rateLimiter16(longitudinalCmd, rate, &state->longitudinalRateFixdt);
  filtLowPass32(state->steeringRateFixdt >> 4, FILTER, &state->steeringFixdt);
  filtLowPass32(state->longitudinalRateFixdt >> 4, FILTER, &state->longitudinalFixdt);

  *steeringIntent = (int16_t)(state->steeringFixdt >> 16);
  *longitudinalIntent = (int16_t)(state->longitudinalFixdt >> 16);
}
