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

#define INTENT_STATE_MACHINE_CMD_DEADBAND        35
#define INTENT_STATE_MACHINE_NEAR_ZERO_ENTER     35
#define INTENT_STATE_MACHINE_NEAR_ZERO_EXIT      50
#define INTENT_STATE_MACHINE_ZERO_LATCH_HOLD_MS  500U
#define INTENT_STATE_MACHINE_TICK_MS             5U

static int8_t IntentStateMachine_Sign16(int16_t value) {
  if (value > 0) {
    return 1;
  }
  if (value < 0) {
    return -1;
  }
  return 0;
}

static int16_t IntentStateMachine_ApplyCommandDeadband(int16_t cmd) {
  if (abs(cmd) < INTENT_STATE_MACHINE_CMD_DEADBAND) {
    return 0;
  }
  return cmd;
}

static void IntentStateMachine_UpdateNearZero(IntentStateMachineState *state, int16_t speed_actual) {
  int16_t speed_abs = (int16_t)abs(speed_actual);

  if (state->near_zero == 0U) {
    if (speed_abs <= INTENT_STATE_MACHINE_NEAR_ZERO_ENTER) {
      state->near_zero = 1U;
    }
  } else {
    if (speed_abs >= INTENT_STATE_MACHINE_NEAR_ZERO_EXIT) {
      state->near_zero = 0U;
    }
  }
}

void IntentStateMachine_Init(IntentStateMachineState *state) {
  IntentStateMachine_Reset(state);
}

void IntentStateMachine_Reset(IntentStateMachineState *state) {
  if (state == 0) {
    return;
  }

  state->mode = INTENT_STATE_MACHINE_DRIVE_FORWARD;
  state->armed_sign = 0;
  state->blocked_sign = 0;
  state->near_zero = 1U;
  state->zero_latch_elapsed_ms = 0U;
}

void IntentStateMachine_Update(IntentStateMachineState *state,
                               int16_t longitudinal_cmd,
                               int16_t speed_actual,
                               IntentStateMachineOutput *output) {
  int16_t cmd_eff;
  int8_t cmd_sign;
  int8_t speed_sign;

  if ((state == 0) || (output == 0)) {
    return;
  }

  output->zero_latch_released = 0U;
  output->zero_latch_armed = 0U;
  output->zero_latch_activated = 0U;

  cmd_eff = IntentStateMachine_ApplyCommandDeadband(longitudinal_cmd);
  cmd_sign = IntentStateMachine_Sign16(cmd_eff);
  speed_sign = IntentStateMachine_Sign16(speed_actual);

  IntentStateMachine_UpdateNearZero(state, speed_actual);

  if ((state->blocked_sign == 0) &&
      (cmd_sign != 0) &&
      (speed_sign != 0) &&
      (cmd_sign != speed_sign) &&
      (abs(speed_actual) > INTENT_STATE_MACHINE_NEAR_ZERO_EXIT)) {
    state->armed_sign = cmd_sign;
    output->zero_latch_armed = 1U;
  }

  if ((state->armed_sign != 0) &&
      (speed_sign == state->armed_sign) &&
      (abs(speed_actual) > INTENT_STATE_MACHINE_NEAR_ZERO_EXIT)) {
    state->armed_sign = 0;
  }

  if ((state->blocked_sign == 0) && (state->armed_sign != 0) && (state->near_zero != 0U)) {
    state->blocked_sign = state->armed_sign;
    state->armed_sign = 0;
    state->zero_latch_elapsed_ms = 0U;
    output->zero_latch_activated = 1U;
  }

  if (state->blocked_sign != 0) {
    if (state->near_zero != 0U) {
      if (state->zero_latch_elapsed_ms < INTENT_STATE_MACHINE_ZERO_LATCH_HOLD_MS) {
        state->zero_latch_elapsed_ms = (uint16_t)(state->zero_latch_elapsed_ms + INTENT_STATE_MACHINE_TICK_MS);
      }
    } else {
      state->zero_latch_elapsed_ms = 0U;
    }

    if ((state->near_zero != 0U) && (state->zero_latch_elapsed_ms >= INTENT_STATE_MACHINE_ZERO_LATCH_HOLD_MS)) {
      state->blocked_sign = 0;
      state->zero_latch_elapsed_ms = 0U;
      output->zero_latch_released = 1U;
    } else if (cmd_sign != state->blocked_sign) {
      state->blocked_sign = 0;
      state->zero_latch_elapsed_ms = 0U;
      output->zero_latch_released = 1U;
    }
  }

  if (state->blocked_sign != 0) {
    output->velocity_intent = 0;
    state->mode = INTENT_STATE_MACHINE_ZERO_LATCH;
  } else {
    output->velocity_intent = cmd_eff;
    state->mode = (cmd_eff < 0) ? INTENT_STATE_MACHINE_DRIVE_REVERSE : INTENT_STATE_MACHINE_DRIVE_FORWARD;
  }

  output->cmd_eff = cmd_eff;
  output->mode = state->mode;
  output->armed_sign = state->armed_sign;
  output->blocked_sign = state->blocked_sign;
  output->near_zero = state->near_zero;
  output->zero_latch_elapsed_ms = state->zero_latch_elapsed_ms;
}
