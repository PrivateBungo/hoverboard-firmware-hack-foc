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

#include "command_filter.h"

void CommandFilter_Init(CommandFilterState *state) {
  CommandFilter_Reset(state);
}

void CommandFilter_Reset(CommandFilterState *state) {
  if (state == 0) {
    return;
  }
  state->reserved = 0U;
}

void CommandFilter_Process(CommandFilterState *state,
                           int16_t steering_cmd,
                           int16_t longitudinal_cmd,
                           CommandFilterOutput *output) {
  (void)state;

  output->steering_cmd = steering_cmd;
  output->longitudinal_cmd = longitudinal_cmd;
}
