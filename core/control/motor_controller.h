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

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <stdint.h>

typedef struct {
  int32_t integrator_q15;
  int16_t torque_cmd;
} MotorControllerState;

typedef struct {
  int16_t torque_cmd;
  int16_t speed_error_rpm;
  uint8_t saturated;
} MotorControllerOutput;

void MotorController_Init(MotorControllerState *state);
void MotorController_Reset(MotorControllerState *state);

void MotorController_Update(MotorControllerState *state,
                            int16_t velocity_setpoint_cmd,
                            int16_t speed_measured_rpm,
                            int16_t speed_max_rpm,
                            uint8_t control_enabled,
                            MotorControllerOutput *output);

#endif // MOTOR_CONTROLLER_H
