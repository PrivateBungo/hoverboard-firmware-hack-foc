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

#include "intent_state_machine.h"

void IntentStateMachine_Init(IntentStateMachineState *state) {
  IntentStateMachine_Reset(state);
}

void IntentStateMachine_Reset(IntentStateMachineState *state) {
  if (state == 0) {
    return;
  }

  state->mode = INTENT_STATE_MACHINE_DRIVE;
}

void IntentStateMachine_Update(IntentStateMachineState *state,
                               int16_t longitudinal_cmd,
                               IntentStateMachineOutput *output) {
  output->velocity_intent = longitudinal_cmd;
  output->mode = state->mode;
}
