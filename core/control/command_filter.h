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

#ifndef COMMAND_FILTER_H
#define COMMAND_FILTER_H

#include <stdint.h>

typedef struct {
  int16_t offset;
  int16_t stable_ref;
  int32_t boot_sum;
  uint16_t stable_count;
  uint16_t boot_elapsed_ms;
  uint16_t boot_samples;
  uint8_t learning_zone;
  uint8_t locked;
  uint8_t boot_calibrated;
} CommandFilterAxisState;

typedef struct {
  CommandFilterAxisState steering;
  CommandFilterAxisState longitudinal;
} CommandFilterState;

typedef struct {
  int16_t steering_raw;
  int16_t longitudinal_raw;
  int16_t steering_cmd;
  int16_t longitudinal_cmd;
  int16_t steering_offset;
  int16_t longitudinal_offset;
  uint8_t longitudinal_calib_active;
  uint8_t longitudinal_calib_locked;
  uint8_t longitudinal_calib_updated;
  uint8_t longitudinal_calib_inhibit_torque;
} CommandFilterOutput;

void CommandFilter_Init(CommandFilterState *state);
void CommandFilter_Reset(CommandFilterState *state);
void CommandFilter_Process(CommandFilterState *state,
                           int16_t steering_cmd,
                           int16_t longitudinal_cmd,
                           CommandFilterOutput *output);

#endif // COMMAND_FILTER_H
