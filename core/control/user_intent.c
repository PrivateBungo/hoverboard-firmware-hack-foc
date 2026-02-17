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

#define USER_INTENT_CMD_MIN       (-1000)
#define USER_INTENT_CMD_MAX       (1000)
#define USER_INTENT_FILTER_SHIFT  (16)

static int16_t UserIntent_ClampS16(int32_t value, int16_t lower, int16_t upper) {
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return (int16_t)value;
}

static int16_t UserIntent_LpfStep(int16_t inputCmd, int32_t *stateFixdt) {
  int32_t xFixdt;

  if (stateFixdt == 0) {
    return inputCmd;
  }

  xFixdt = ((int32_t)inputCmd) << USER_INTENT_FILTER_SHIFT;
  *stateFixdt += (((xFixdt - *stateFixdt) * USER_INTENT_INPUT_FILTER_COEF) >> USER_INTENT_FILTER_SHIFT);

  return (int16_t)(*stateFixdt >> USER_INTENT_FILTER_SHIFT);
}

void UserIntent_Init(UserIntentState *state) {
  UserIntent_Reset(state);
}

void UserIntent_Reset(UserIntentState *state) {
  if (state == 0) {
    return;
  }
  state->steeringFiltFixdt = 0;
  state->longitudinalFiltFixdt = 0;
}

void UserIntent_BuildLongitudinalSteeringIntent(UserIntentState *state,
                                                int16_t steeringCmd,
                                                int16_t longitudinalCmd,
                                                int16_t *steeringIntent,
                                                int16_t *longitudinalIntent) {
  int16_t steeringFiltered;
  int16_t longitudinalFiltered;
  int16_t steeringOut;
  int16_t longitudinalOut;
  int32_t boostedSteer;

  steeringFiltered = UserIntent_LpfStep(steeringCmd, (state != 0) ? &state->steeringFiltFixdt : 0);
  longitudinalFiltered = UserIntent_LpfStep(longitudinalCmd, (state != 0) ? &state->longitudinalFiltFixdt : 0);

  steeringOut = steeringFiltered;
  longitudinalOut = longitudinalFiltered;

  if ((longitudinalFiltered >= -USER_INTENT_ZERO_DEADBAND) && (longitudinalFiltered <= USER_INTENT_ZERO_DEADBAND)) {
    longitudinalOut = 0;

    if ((steeringFiltered >= USER_INTENT_ZERO_RELEASE_STEER_THRESH) ||
        (steeringFiltered <= -USER_INTENT_ZERO_RELEASE_STEER_THRESH)) {
      boostedSteer = ((int32_t)steeringFiltered * USER_INTENT_ZERO_RELEASE_STEER_GAIN_NUM) / USER_INTENT_ZERO_RELEASE_STEER_GAIN_DEN;
      steeringOut = UserIntent_ClampS16(boostedSteer,
                                        USER_INTENT_CMD_MIN,
                                        USER_INTENT_CMD_MAX);
    }
  }

  *steeringIntent = steeringOut;
  *longitudinalIntent = longitudinalOut;
}
