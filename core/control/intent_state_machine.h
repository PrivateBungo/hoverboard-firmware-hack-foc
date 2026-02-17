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

#ifndef INTENT_STATE_MACHINE_H
#define INTENT_STATE_MACHINE_H

#include <stdint.h>

typedef enum {
  INTENT_STATE_MACHINE_DRIVE = 0,
} IntentStateMachineMode;

typedef struct {
  IntentStateMachineMode mode;
} IntentStateMachineState;

typedef struct {
  int16_t velocity_intent;
  IntentStateMachineMode mode;
} IntentStateMachineOutput;

void IntentStateMachine_Init(IntentStateMachineState *state);
void IntentStateMachine_Reset(IntentStateMachineState *state);
void IntentStateMachine_Update(IntentStateMachineState *state,
                               int16_t longitudinal_cmd,
                               IntentStateMachineOutput *output);

#endif // INTENT_STATE_MACHINE_H
