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

#include "drive_control.h"
#include "drive_math.h"
#include "config.h"

void DriveControl_Init(DriveControlState *state) {
  DriveControl_ResetFilters(state);
}

void DriveControl_ResetFilters(DriveControlState *state) {
  state->steerRateFixdt = 0;
  state->speedRateFixdt = 0;
  state->steerFixdt = 0;
  state->speedFixdt = 0;
}

void DriveControl_FilterInputs(DriveControlState *state, int16_t steerCmd, int16_t speedCmd, uint16_t rate, int16_t *steer, int16_t *speed) {
  rateLimiter16(steerCmd, rate, &state->steerRateFixdt);
  rateLimiter16(speedCmd, rate, &state->speedRateFixdt);
  filtLowPass32(state->steerRateFixdt >> 4, FILTER, &state->steerFixdt);
  filtLowPass32(state->speedRateFixdt >> 4, FILTER, &state->speedFixdt);

  *steer = (int16_t)(state->steerFixdt >> 16);
  *speed = (int16_t)(state->speedFixdt >> 16);
}

void DriveControl_MixCommands(int16_t speed, int16_t steer, int16_t *cmdL, int16_t *cmdR) {
#if defined(TANK_STEERING) && !defined(VARIANT_HOVERCAR) && !defined(VARIANT_SKATEBOARD)
  *cmdL = steer;
  *cmdR = speed;
#else
  mixerFcn(speed << 4, steer << 4, cmdR, cmdL);
#endif
}

void DriveControl_MapCommandsToPwm(int16_t cmdL, int16_t cmdR, volatile int *pwml, volatile int *pwmr) {
#ifdef INVERT_R_DIRECTION
  *pwmr = cmdR;
#else
  *pwmr = -cmdR;
#endif
#ifdef INVERT_L_DIRECTION
  *pwml = -cmdL;
#else
  *pwml = cmdL;
#endif
}

void DriveControl_ResetStallDecay(DriveControlStallDecayState *state) {
  state->stallTimerMs = 0;
}

int16_t DriveControl_ApplyStallDecay(int16_t torqueCmd, int16_t wheelSpeedRpm, uint8_t isTorqueMode, DriveControlStallDecayState *state) {
  int16_t cmdAbs;
  int16_t speedAbs;
  int16_t cmdSign;
  int32_t cmdMax;

  if (!isTorqueMode) {
    state->stallTimerMs = 0;
    return torqueCmd;
  }

  cmdSign = (torqueCmd >= 0) ? 1 : -1;
  cmdAbs = (torqueCmd >= 0) ? torqueCmd : (int16_t)-torqueCmd;
  speedAbs = (wheelSpeedRpm >= 0) ? wheelSpeedRpm : (int16_t)-wheelSpeedRpm;

  if ((speedAbs <= STALL_DECAY_SPEED_RPM) && (cmdAbs >= STALL_DECAY_CMD_TRIGGER)) {
    if (state->stallTimerMs < STALL_DECAY_TIME_MS) {
      uint16_t stepMs = DELAY_IN_MAIN_LOOP + 1U;
      uint16_t nextMs = state->stallTimerMs + stepMs;
      state->stallTimerMs = (nextMs < STALL_DECAY_TIME_MS) ? nextMs : STALL_DECAY_TIME_MS;
    }
  } else {
    state->stallTimerMs = 0;
    return torqueCmd;
  }

  if (state->stallTimerMs >= STALL_DECAY_PREEMPT_MS) {
    cmdMax = STALL_DECAY_CMD_PREEMPT;
  } else {
    cmdMax = 1000 - (((int32_t)(1000 - STALL_DECAY_CMD_PREEMPT) * state->stallTimerMs) / STALL_DECAY_PREEMPT_MS);
  }

  if (state->stallTimerMs >= STALL_DECAY_TIME_MS) {
    cmdMax = STALL_DECAY_CMD_FLOOR;
  } else if (state->stallTimerMs > STALL_DECAY_PREEMPT_MS) {
    uint16_t decayWindowMs = STALL_DECAY_TIME_MS - STALL_DECAY_PREEMPT_MS;
    uint16_t elapsedDecayMs = state->stallTimerMs - STALL_DECAY_PREEMPT_MS;
    cmdMax = STALL_DECAY_CMD_PREEMPT - (((int32_t)(STALL_DECAY_CMD_PREEMPT - STALL_DECAY_CMD_FLOOR) * elapsedDecayMs) / decayWindowMs);
  }

  if (cmdMax < STALL_DECAY_CMD_FLOOR) {
    cmdMax = STALL_DECAY_CMD_FLOOR;
  }

  if (cmdAbs > cmdMax) {
    cmdAbs = (int16_t)cmdMax;
  }

  return (int16_t)(cmdAbs * cmdSign);
}
