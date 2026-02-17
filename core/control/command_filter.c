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
#include "command_filter.h"

#define COMMAND_FILTER_LEARN_ZONE_ENTER          35
#define COMMAND_FILTER_LEARN_ZONE_EXIT           50
#define COMMAND_FILTER_LEARN_OPERATOR_ABORT     300
#define COMMAND_FILTER_LEARN_STABLE_DELTA         8
#define COMMAND_FILTER_LEARN_STABLE_COUNT_MIN    40U
#define COMMAND_FILTER_OFFSET_MAX               200

static int16_t CommandFilter_ClampOffset(int16_t offset) {
  if (offset > COMMAND_FILTER_OFFSET_MAX) {
    return COMMAND_FILTER_OFFSET_MAX;
  }
  if (offset < -COMMAND_FILTER_OFFSET_MAX) {
    return -COMMAND_FILTER_OFFSET_MAX;
  }
  return offset;
}

static void CommandFilter_UpdateOffset(int16_t raw,
                                       int16_t *offset,
                                       int16_t *stableRef,
                                       uint16_t *stableCount) {
  int16_t centered;
  int16_t centeredAbs;

  centered = (int16_t)(raw - *offset);
  centeredAbs = (int16_t)abs(centered);

  if (abs(raw) > COMMAND_FILTER_LEARN_OPERATOR_ABORT) {
    *stableCount = 0U;
    *stableRef = raw;
    return;
  }

  if (centeredAbs >= COMMAND_FILTER_LEARN_ZONE_EXIT) {
    *stableCount = 0U;
    *stableRef = raw;
    return;
  }

  if (centeredAbs <= COMMAND_FILTER_LEARN_ZONE_ENTER) {
    int16_t delta = (int16_t)abs(raw - *stableRef);

    if (delta <= COMMAND_FILTER_LEARN_STABLE_DELTA) {
      if (*stableCount < 0xFFFFU) {
        *stableCount = (uint16_t)(*stableCount + 1U);
      }
    } else {
      *stableCount = 0U;
      *stableRef = raw;
    }

    if (*stableCount >= COMMAND_FILTER_LEARN_STABLE_COUNT_MIN) {
      int16_t targetOffset = (int16_t)(((*offset * 7) + raw) / 8);
      *offset = CommandFilter_ClampOffset(targetOffset);
      *stableCount = 0U;
      *stableRef = raw;
    }
  }
}

void CommandFilter_Init(CommandFilterState *state) {
  CommandFilter_Reset(state);
}

void CommandFilter_Reset(CommandFilterState *state) {
  if (state == 0) {
    return;
  }

  state->steering_offset = 0;
  state->longitudinal_offset = 0;
  state->steering_stable_ref = 0;
  state->longitudinal_stable_ref = 0;
  state->steering_stable_count = 0U;
  state->longitudinal_stable_count = 0U;
}

void CommandFilter_Process(CommandFilterState *state,
                           int16_t steering_cmd,
                           int16_t longitudinal_cmd,
                           CommandFilterOutput *output) {
  if ((state == 0) || (output == 0)) {
    return;
  }

  CommandFilter_UpdateOffset(steering_cmd,
                             &state->steering_offset,
                             &state->steering_stable_ref,
                             &state->steering_stable_count);

  CommandFilter_UpdateOffset(longitudinal_cmd,
                             &state->longitudinal_offset,
                             &state->longitudinal_stable_ref,
                             &state->longitudinal_stable_count);

  output->steering_offset = state->steering_offset;
  output->longitudinal_offset = state->longitudinal_offset;
  output->steering_cmd = (int16_t)(steering_cmd - state->steering_offset);
  output->longitudinal_cmd = (int16_t)(longitudinal_cmd - state->longitudinal_offset);
}
