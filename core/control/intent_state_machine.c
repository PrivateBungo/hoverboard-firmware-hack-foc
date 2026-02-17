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
#include "intent_state_machine.h"

#define INTENT_STATE_MACHINE_ZERO_LATCH_SPEED_DEADBAND  60
#define INTENT_STATE_MACHINE_ZERO_LATCH_HOLD_MS         500U
#define INTENT_STATE_MACHINE_TICK_MS                    5U

static int16_t IntentStateMachine_ClampToIntentSign(IntentStateMachineMode mode, int16_t longitudinal_cmd) {
  if (mode == INTENT_STATE_MACHINE_DRIVE_FORWARD && longitudinal_cmd < 0) {
    return 0;
  }
  if (mode == INTENT_STATE_MACHINE_DRIVE_REVERSE && longitudinal_cmd > 0) {
    return 0;
  }
  return longitudinal_cmd;
}

void IntentStateMachine_Init(IntentStateMachineState *state) {
  IntentStateMachine_Reset(state);
}

void IntentStateMachine_Reset(IntentStateMachineState *state) {
  if (state == 0) {
    return;
  }

  state->mode = INTENT_STATE_MACHINE_DRIVE_FORWARD;
  state->latch_target_mode = INTENT_STATE_MACHINE_DRIVE_FORWARD;
  state->zero_latch_elapsed_ms = 0U;
}

void IntentStateMachine_Update(IntentStateMachineState *state,
                               int16_t longitudinal_cmd,
                               int16_t speed_actual,
                               IntentStateMachineOutput *output) {
  int16_t speedAbs = (int16_t)abs(speed_actual);

  output->zero_latch_released = 0U;

  if (state->mode == INTENT_STATE_MACHINE_DRIVE_FORWARD) {
    if (longitudinal_cmd < 0) {
      if (speedAbs <= INTENT_STATE_MACHINE_ZERO_LATCH_SPEED_DEADBAND) {
        state->mode = INTENT_STATE_MACHINE_ZERO_LATCH;
        state->latch_target_mode = INTENT_STATE_MACHINE_DRIVE_REVERSE;
        state->zero_latch_elapsed_ms = 0U;
      } else {
        state->mode = INTENT_STATE_MACHINE_DRIVE_REVERSE;
      }
    }
  } else if (state->mode == INTENT_STATE_MACHINE_DRIVE_REVERSE) {
    if (longitudinal_cmd > 0) {
      if (speedAbs <= INTENT_STATE_MACHINE_ZERO_LATCH_SPEED_DEADBAND) {
        state->mode = INTENT_STATE_MACHINE_ZERO_LATCH;
        state->latch_target_mode = INTENT_STATE_MACHINE_DRIVE_FORWARD;
        state->zero_latch_elapsed_ms = 0U;
      } else {
        state->mode = INTENT_STATE_MACHINE_DRIVE_FORWARD;
      }
    }
  }

  if (state->mode == INTENT_STATE_MACHINE_ZERO_LATCH) {
    if ((state->latch_target_mode == INTENT_STATE_MACHINE_DRIVE_FORWARD && longitudinal_cmd < 0) ||
        (state->latch_target_mode == INTENT_STATE_MACHINE_DRIVE_REVERSE && longitudinal_cmd > 0)) {
      state->mode = (state->latch_target_mode == INTENT_STATE_MACHINE_DRIVE_FORWARD)
                    ? INTENT_STATE_MACHINE_DRIVE_REVERSE
                    : INTENT_STATE_MACHINE_DRIVE_FORWARD;
      state->zero_latch_elapsed_ms = 0U;
      output->zero_latch_released = 1U;
    } else {
      if (speedAbs <= INTENT_STATE_MACHINE_ZERO_LATCH_SPEED_DEADBAND) {
        if (state->zero_latch_elapsed_ms < INTENT_STATE_MACHINE_ZERO_LATCH_HOLD_MS) {
          state->zero_latch_elapsed_ms = (uint16_t)(state->zero_latch_elapsed_ms + INTENT_STATE_MACHINE_TICK_MS);
        }
      } else {
        state->zero_latch_elapsed_ms = 0U;
      }

      if (state->zero_latch_elapsed_ms >= INTENT_STATE_MACHINE_ZERO_LATCH_HOLD_MS) {
        state->mode = state->latch_target_mode;
        state->zero_latch_elapsed_ms = 0U;
        output->zero_latch_released = 1U;
      }
    }
  }

  if (state->mode == INTENT_STATE_MACHINE_ZERO_LATCH) {
    output->velocity_intent = 0;
  } else {
    output->velocity_intent = IntentStateMachine_ClampToIntentSign(state->mode, longitudinal_cmd);
  }

  output->mode = state->mode;
  output->zero_latch_elapsed_ms = state->zero_latch_elapsed_ms;
}
