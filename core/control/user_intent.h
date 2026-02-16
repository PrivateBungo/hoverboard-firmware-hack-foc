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

#ifndef USER_INTENT_H
#define USER_INTENT_H

#include <stdint.h>

typedef struct {
  uint8_t reserved;
} UserIntentState;

void UserIntent_Init(UserIntentState *state);
void UserIntent_Reset(UserIntentState *state);
void UserIntent_BuildLongitudinalSteeringIntent(UserIntentState *state,
                                                int16_t steeringCmd,
                                                int16_t longitudinalCmd,
                                                int16_t *steeringIntent,
                                                int16_t *longitudinalIntent);

#endif // USER_INTENT_H
