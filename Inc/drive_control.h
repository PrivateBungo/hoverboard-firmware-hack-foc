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
  uint16_t stallTimerMs;
} DriveControlStallDecayState;

void DriveControl_MixCommands(int16_t speed, int16_t steer, int16_t *cmdL, int16_t *cmdR);
void DriveControl_MapCommandsToPwm(int16_t cmdL, int16_t cmdR, volatile int *pwml, volatile int *pwmr);
void DriveControl_ResetStallDecay(DriveControlStallDecayState *state);
int16_t DriveControl_ApplySlipSoftLimit(int16_t torqueCmd, uint8_t slipGapClampActive, int16_t softLimit);
int16_t DriveControl_ApplyStallDecay(int16_t torqueCmd, int16_t wheelSpeedRpm, uint8_t isTorqueMode, DriveControlStallDecayState *state);

#endif // DRIVE_CONTROL_H
