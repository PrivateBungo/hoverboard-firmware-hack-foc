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


int16_t DriveControl_ApplySoftTorqueLimit(int16_t torqueCmd, int16_t wheelSpeedAbsRpm, uint8_t isTorqueMode) {
  int32_t softCurrentA;
  int32_t speedForRamp;

  if (!isTorqueMode) {
    return torqueCmd;
  }

#if (I_MOT_MAX <= 0)
  return 0;
#else
  if (I_AC_SOFT_RPM <= 0) {
    softCurrentA = I_MOT_MAX;
  } else {
    speedForRamp = wheelSpeedAbsRpm;
    if (speedForRamp < 0) {
      speedForRamp = 0;
    } else if (speedForRamp > I_AC_SOFT_RPM) {
      speedForRamp = I_AC_SOFT_RPM;
    }
    softCurrentA = I_AC_SOFT_MAX + (((int32_t)(I_MOT_MAX - I_AC_SOFT_MAX) * speedForRamp) / I_AC_SOFT_RPM);
  }

  if (softCurrentA < 0) {
    softCurrentA = 0;
  } else if (softCurrentA > I_MOT_MAX) {
    softCurrentA = I_MOT_MAX;
  }
  return (int16_t)(((int32_t)torqueCmd * softCurrentA) / I_MOT_MAX);
#endif
}
