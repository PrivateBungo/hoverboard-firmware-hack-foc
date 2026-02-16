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

void UserIntent_Init(UserIntentState *state) {
  UserIntent_Reset(state);
}

void UserIntent_Reset(UserIntentState *state) {
  if (state == 0) {
    return;
  }
  state->reserved = 0U;
}

void UserIntent_BuildLongitudinalSteeringIntent(UserIntentState *state,
                                                int16_t steeringCmd,
                                                int16_t longitudinalCmd,
                                                int16_t *steeringIntent,
                                                int16_t *longitudinalIntent) {
  (void)state;

  *steeringIntent = steeringCmd;
  *longitudinalIntent = longitudinalCmd;
}
