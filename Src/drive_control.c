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
#include "config.h"
#include "drive_math.h"

static int16_t DriveControl_ClampS16(int32_t value, int16_t lower, int16_t upper) {
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return (int16_t)value;
}

int16_t DriveControl_ApplySlipSoftLimit(int16_t torqueCmd, uint8_t slipGapClampActive, int16_t softLimit) {
  int16_t torqueLimit = (softLimit >= 0) ? softLimit : (int16_t)-softLimit;

  if (slipGapClampActive == 0U) {
    return torqueCmd;
  }

  if (torqueLimit <= 0) {
    return 0;
  }

  return DriveControl_ClampS16(torqueCmd, (int16_t)-torqueLimit, torqueLimit);
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
  uint16_t preemptMs;
  uint16_t totalMs;

  if (!isTorqueMode) {
    state->stallTimerMs = 0;
    return torqueCmd;
  }

  cmdSign = (torqueCmd >= 0) ? 1 : -1;
  cmdAbs = (torqueCmd >= 0) ? torqueCmd : (int16_t)-torqueCmd;
  speedAbs = (wheelSpeedRpm >= 0) ? wheelSpeedRpm : (int16_t)-wheelSpeedRpm;

  preemptMs = (STALL_DECAY_PREEMPT_MS > 0U) ? STALL_DECAY_PREEMPT_MS : 1U;
  totalMs = (STALL_DECAY_TIME_MS > preemptMs) ? STALL_DECAY_TIME_MS : (uint16_t)(preemptMs + 1U);

  if ((speedAbs <= STALL_DECAY_SPEED_RPM) && (cmdAbs >= STALL_DECAY_CMD_TRIGGER)) {
    if (state->stallTimerMs < totalMs) {
      uint16_t stepMs = DELAY_IN_MAIN_LOOP + 1U;
      uint16_t nextMs = state->stallTimerMs + stepMs;
      state->stallTimerMs = (nextMs < totalMs) ? nextMs : totalMs;
    }
  } else {
    state->stallTimerMs = 0;
    return torqueCmd;
  }

  if (state->stallTimerMs >= preemptMs) {
    cmdMax = STALL_DECAY_CMD_PREEMPT;
  } else {
    cmdMax = 1000 - (((int32_t)(1000 - STALL_DECAY_CMD_PREEMPT) * state->stallTimerMs) / preemptMs);
  }

  if (state->stallTimerMs >= totalMs) {
    cmdMax = STALL_DECAY_CMD_FLOOR;
  } else if (state->stallTimerMs > preemptMs) {
    uint16_t decayWindowMs = totalMs - preemptMs;
    uint16_t elapsedDecayMs = state->stallTimerMs - preemptMs;
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
