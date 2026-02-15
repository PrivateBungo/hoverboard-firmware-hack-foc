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

#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include <stdint.h>

typedef struct {
  int16_t steerRateFixdt;
  int16_t speedRateFixdt;
  int32_t steerFixdt;
  int32_t speedFixdt;
  int16_t prevWheelSpeedL;
  int16_t prevWheelSpeedR;
  float accelFiltL;
  float accelFiltR;
  int16_t prevEffectiveCmdL;
  int16_t prevEffectiveCmdR;
  uint8_t torqueSupervisorInitialized;
} DriveControlState;

void DriveControl_Init(DriveControlState *state);
void DriveControl_ResetFilters(DriveControlState *state);
void DriveControl_FilterInputs(DriveControlState *state, int16_t steerCmd, int16_t speedCmd, uint16_t rate, int16_t *steer, int16_t *speed);
void DriveControl_MixCommands(int16_t speed, int16_t steer, int16_t *cmdL, int16_t *cmdR);
void DriveControl_ApplyTorqueModeAdjustments(DriveControlState *state, uint8_t torqueModeActive, float dtSec, int16_t speedLMeasRpm, int16_t speedRMeasRpm, int16_t *cmdL, int16_t *cmdR);
void DriveControl_MapCommandsToPwm(int16_t cmdL, int16_t cmdR, volatile int *pwml, volatile int *pwmr);

#endif // DRIVE_CONTROL_H
