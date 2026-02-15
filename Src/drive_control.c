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

static float clampf(float value, float minVal, float maxVal) {
  if (value < minVal) {
    return minVal;
  }
  if (value > maxVal) {
    return maxVal;
  }
  return value;
}

static void DriveControl_ResetTorqueSupervisor(DriveControlState *state, int16_t speedLMeasRpm, int16_t speedRMeasRpm) {
  state->prevWheelSpeedL = speedLMeasRpm;
  state->prevWheelSpeedR = speedRMeasRpm;
  state->accelFiltL = 0.0f;
  state->accelFiltR = 0.0f;
  state->prevEffectiveCmdL = 0;
  state->prevEffectiveCmdR = 0;
  state->stallTimerL = 0.0f;
  state->stallTimerR = 0.0f;
  state->torqueSupervisorInitialized = 1;
}

static int16_t DriveControl_ApplyAccelLimitWheel(int16_t cmdReq, int16_t cmdPrev, float accelFilt, float accelLimitRpmPerSec, float releaseRatePerSec, float dtSec) {
  int16_t cmdOut = cmdReq;

  if ((cmdReq > 0 && accelFilt > accelLimitRpmPerSec) || (cmdReq < 0 && accelFilt < -accelLimitRpmPerSec)) {
    cmdOut = 0;
  }

  if ((cmdOut > 0 && cmdOut > cmdPrev) || (cmdOut < 0 && cmdOut < cmdPrev)) {
    const float maxStep = releaseRatePerSec * dtSec;
    if (cmdOut > 0) {
      const float limited = ((float)cmdPrev + maxStep < (float)cmdOut) ? ((float)cmdPrev + maxStep) : (float)cmdOut;
      cmdOut = (int16_t)limited;
    } else {
      const float limited = ((float)cmdPrev - maxStep > (float)cmdOut) ? ((float)cmdPrev - maxStep) : (float)cmdOut;
      cmdOut = (int16_t)limited;
    }
  }

  return cmdOut;
}

static int16_t DriveControl_ApplyStallClampWheel(int16_t cmdIn, int16_t speedMeasRpm, float *stallTimerSec, float dtSec) {
  int16_t cmdOut = cmdIn;
  const int16_t cmdAbs = ABS(cmdIn);
  const int16_t speedAbs = ABS(speedMeasRpm);

  if (cmdAbs >= TORQUE_STALL_CMD_THRESHOLD && speedAbs <= TORQUE_STALL_SPEED_RPM) {
    *stallTimerSec += dtSec;
  } else if (speedAbs > TORQUE_STALL_SPEED_RPM + 5 || cmdAbs < (TORQUE_STALL_CMD_THRESHOLD - 20)) {
    *stallTimerSec = 0.0f;
  }

  if (*stallTimerSec >= TORQUE_STALL_HOLD_TIME_SEC) {
    if (cmdOut > TORQUE_STALL_CMD_CLAMP) {
      cmdOut = TORQUE_STALL_CMD_CLAMP;
    } else if (cmdOut < -TORQUE_STALL_CMD_CLAMP) {
      cmdOut = -TORQUE_STALL_CMD_CLAMP;
    }
  }

  return cmdOut;
}

void DriveControl_Init(DriveControlState *state) {
  DriveControl_ResetFilters(state);
}

void DriveControl_ResetFilters(DriveControlState *state) {
  state->steerRateFixdt = 0;
  state->speedRateFixdt = 0;
  state->steerFixdt = 0;
  state->speedFixdt = 0;
  state->prevWheelSpeedL = 0;
  state->prevWheelSpeedR = 0;
  state->accelFiltL = 0.0f;
  state->accelFiltR = 0.0f;
  state->prevEffectiveCmdL = 0;
  state->prevEffectiveCmdR = 0;
  state->stallTimerL = 0.0f;
  state->stallTimerR = 0.0f;
  state->torqueSupervisorInitialized = 0;
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

void DriveControl_ApplyTorqueModeAdjustments(DriveControlState *state, uint8_t torqueModeActive, float dtSec, int16_t speedLMeasRpm, int16_t speedRMeasRpm, int16_t *cmdL, int16_t *cmdR) {
  if (!torqueModeActive || dtSec <= 0.0f) {
    state->torqueSupervisorInitialized = 0;
  } else {
    if (!state->torqueSupervisorInitialized) {
      DriveControl_ResetTorqueSupervisor(state, speedLMeasRpm, speedRMeasRpm);
    }

    const float accelRawL = ((float)speedLMeasRpm - (float)state->prevWheelSpeedL) / dtSec;
    const float accelRawR = ((float)speedRMeasRpm - (float)state->prevWheelSpeedR) / dtSec;

    state->prevWheelSpeedL = speedLMeasRpm;
    state->prevWheelSpeedR = speedRMeasRpm;

    const float tauSec = TORQUE_ACCEL_SUPERVISOR_LPF_TAU_SEC;
    const float alpha = clampf(dtSec / (tauSec + dtSec), 0.0f, 1.0f);
    state->accelFiltL += alpha * (accelRawL - state->accelFiltL);
    state->accelFiltR += alpha * (accelRawR - state->accelFiltR);

    if (TORQUE_ACCEL_SUPERVISOR_ENABLE) {
      const float releaseRatePerSec = (float)TORQUE_ACCEL_LIMIT_RPM_PER_SEC;
      *cmdL = DriveControl_ApplyAccelLimitWheel(*cmdL, state->prevEffectiveCmdL, state->accelFiltL, TORQUE_ACCEL_LIMIT_RPM_PER_SEC, releaseRatePerSec, dtSec);
      *cmdR = DriveControl_ApplyAccelLimitWheel(*cmdR, state->prevEffectiveCmdR, state->accelFiltR, TORQUE_ACCEL_LIMIT_RPM_PER_SEC, releaseRatePerSec, dtSec);
    }

    if (TORQUE_STALL_SUPERVISOR_ENABLE) {
      *cmdL = DriveControl_ApplyStallClampWheel(*cmdL, speedLMeasRpm, &state->stallTimerL, dtSec);
      *cmdR = DriveControl_ApplyStallClampWheel(*cmdR, speedRMeasRpm, &state->stallTimerR, dtSec);
    }
  }

  if (torqueModeActive) {
    const int32_t cmdLWithOffset = (int32_t)(*cmdL) + TORQUE_OFFSET_LEFT;
    const int32_t cmdRWithOffset = (int32_t)(*cmdR) + TORQUE_OFFSET_RIGHT;
    *cmdL = (int16_t)CLAMP(cmdLWithOffset, -1000, 1000);
    *cmdR = (int16_t)CLAMP(cmdRWithOffset, -1000, 1000);
  }

  if (torqueModeActive) {
    state->prevEffectiveCmdL = *cmdL;
    state->prevEffectiveCmdR = *cmdR;
  } else {
    state->prevEffectiveCmdL = 0;
    state->prevEffectiveCmdR = 0;
    state->stallTimerL = 0.0f;
    state->stallTimerR = 0.0f;
  }
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
