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
  INTENT_STATE_MACHINE_DRIVE_FORWARD = 0,
  INTENT_STATE_MACHINE_DRIVE_REVERSE = 1,
  INTENT_STATE_MACHINE_ZERO_LATCH = 2,
} IntentStateMachineMode;

typedef struct {
  IntentStateMachineMode mode;
  IntentStateMachineMode latch_target_mode;
  uint16_t zero_latch_elapsed_ms;
} IntentStateMachineState;

typedef struct {
  int16_t velocity_intent;
  IntentStateMachineMode mode;
  uint16_t zero_latch_elapsed_ms;
  uint8_t zero_latch_released;
} IntentStateMachineOutput;

void IntentStateMachine_Init(IntentStateMachineState *state);
void IntentStateMachine_Reset(IntentStateMachineState *state);
void IntentStateMachine_Update(IntentStateMachineState *state,
                               int16_t longitudinal_cmd,
                               int16_t speed_actual,
                               IntentStateMachineOutput *output);

#endif // INTENT_STATE_MACHINE_H
