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

#include <stdlib.h>
#include "user_intent.h"
#include "config.h"

static int16_t UserIntent_ApplyAxisHysteresis(int16_t input, uint8_t *active) {
  int16_t absInput = (int16_t)abs(input);

  if (*active == 0U) {
    if (absInput >= USER_INTENT_HYST_ON) {
      *active = 1U;
    }
  } else {
    if (absInput <= USER_INTENT_HYST_OFF) {
      *active = 0U;
    }
  }

  if (*active == 0U) {
    return 0;
  }

  return input;
}

void UserIntent_Init(UserIntentState *state) {
  UserIntent_Reset(state);
}

void UserIntent_Reset(UserIntentState *state) {
  if (state == 0) {
    return;
  }
  state->steering_active = 0U;
  state->longitudinal_active = 0U;
}

void UserIntent_BuildLongitudinalSteeringIntent(UserIntentState *state,
                                                int16_t steeringCmd,
                                                int16_t longitudinalCmd,
                                                int16_t *steeringIntent,
                                                int16_t *longitudinalIntent) {
  if ((state == 0) || (steeringIntent == 0) || (longitudinalIntent == 0)) {
    return;
  }

  *steeringIntent = UserIntent_ApplyAxisHysteresis(steeringCmd, &state->steering_active);
  *longitudinalIntent = UserIntent_ApplyAxisHysteresis(longitudinalCmd, &state->longitudinal_active);
}
