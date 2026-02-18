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
#include "config.h"

#define COMMAND_FILTER_TICK_MS  ((uint16_t)(DELAY_IN_MAIN_LOOP + 1U))

static int16_t CommandFilter_ClampOffset(int16_t offset) {
  if (offset > COMMAND_FILTER_OFFSET_MAX) {
    return COMMAND_FILTER_OFFSET_MAX;
  }
  if (offset < -COMMAND_FILTER_OFFSET_MAX) {
    return -COMMAND_FILTER_OFFSET_MAX;
  }
  return offset;
}

static void CommandFilter_ResetAxis(CommandFilterAxisState *axis) {
  axis->offset = 0;
  axis->stable_ref = 0;
  axis->boot_sum = 0;
  axis->stable_count = 0U;
  axis->boot_elapsed_ms = 0U;
  axis->boot_samples = 0U;
  axis->learning_zone = 0U;
  axis->locked = 0U;
  axis->boot_calibrated = 0U;
}

static uint8_t CommandFilter_UpdateBootCalibration(CommandFilterAxisState *axis,
                                                   int16_t raw,
                                                   uint8_t *calibInProgressOut) {
  uint32_t nextElapsed;
  uint16_t sampleStartMs = COMMAND_FILTER_BOOT_SETTLE_MS;
  uint16_t sampleStopMs = (uint16_t)(COMMAND_FILTER_BOOT_SETTLE_MS + COMMAND_FILTER_BOOT_SAMPLE_MS);

  if (axis->boot_calibrated != 0U) {
    *calibInProgressOut = 0U;
    return 0U;
  }

  *calibInProgressOut = 1U;

  nextElapsed = (uint32_t)axis->boot_elapsed_ms + COMMAND_FILTER_TICK_MS;
  axis->boot_elapsed_ms = (nextElapsed > 0xFFFFU) ? 0xFFFFU : (uint16_t)nextElapsed;

  if (axis->boot_elapsed_ms <= sampleStartMs) {
    axis->stable_ref = raw;
    axis->stable_count = 0U;
    return 0U;
  }

  if (axis->boot_elapsed_ms <= sampleStopMs) {
    axis->boot_sum += raw;
    if (axis->boot_samples < 0xFFFFU) {
      axis->boot_samples = (uint16_t)(axis->boot_samples + 1U);
    }
    return 0U;
  }

  if (axis->boot_samples > 0U) {
    axis->offset = CommandFilter_ClampOffset((int16_t)(axis->boot_sum / (int32_t)axis->boot_samples));
  } else {
    axis->offset = CommandFilter_ClampOffset(raw);
  }

  axis->boot_calibrated = 1U;
  axis->locked = 1U;
  axis->learning_zone = 0U;
  axis->stable_ref = raw;
  axis->stable_count = 0U;
  *calibInProgressOut = 0U;

  return 1U;
}

static uint8_t CommandFilter_UpdateAdaptiveOffset(CommandFilterAxisState *axis, int16_t raw) {
  int16_t centered = (int16_t)(raw - axis->offset);
  int16_t centered_abs = (int16_t)abs(centered);
  uint8_t updated = 0U;

  if (abs(raw) > COMMAND_FILTER_LEARN_OPERATOR_ABORT) {
    axis->stable_count = 0U;
    axis->stable_ref = raw;
    axis->learning_zone = 0U;
    return 0U;
  }

  if (axis->learning_zone == 0U) {
    if (centered_abs <= COMMAND_FILTER_LEARN_ZONE_ENTER) {
      axis->learning_zone = 1U;
      axis->stable_ref = raw;
      axis->stable_count = 0U;
    }
  } else {
    if (centered_abs >= COMMAND_FILTER_LEARN_ZONE_EXIT) {
      axis->learning_zone = 0U;
      axis->stable_count = 0U;
      axis->stable_ref = raw;
    }
  }

  if (axis->learning_zone != 0U) {
    if (abs(raw - axis->stable_ref) <= COMMAND_FILTER_LEARN_STABLE_DELTA) {
      if (axis->stable_count < 0xFFFFU) {
        axis->stable_count = (uint16_t)(axis->stable_count + 1U);
      }
    } else {
      axis->stable_ref = raw;
      axis->stable_count = 0U;
    }

    if (axis->stable_count >= COMMAND_FILTER_LEARN_STABLE_COUNT_MIN) {
      int16_t targetOffset = (int16_t)((axis->offset * 7 + raw) / 8);
      int16_t clamped = CommandFilter_ClampOffset(targetOffset);

      updated = (uint8_t)(clamped != axis->offset);
      axis->offset = clamped;
      axis->stable_count = 0U;
      axis->stable_ref = raw;
    }
  }

  return updated;
}

void CommandFilter_Init(CommandFilterState *state) {
  CommandFilter_Reset(state);
}

void CommandFilter_Reset(CommandFilterState *state) {
  if (state == 0) {
    return;
  }

  CommandFilter_ResetAxis(&state->steering);
  CommandFilter_ResetAxis(&state->longitudinal);
}

void CommandFilter_Process(CommandFilterState *state,
                           int16_t steering_cmd,
                           int16_t longitudinal_cmd,
                           CommandFilterOutput *output) {
  uint8_t longBootCalibActive;
  uint8_t steeringBootCalibActive;

  if ((state == 0) || (output == 0)) {
    return;
  }

  output->steering_raw = steering_cmd;
  output->longitudinal_raw = longitudinal_cmd;

  (void)CommandFilter_UpdateBootCalibration(&state->steering, steering_cmd, &steeringBootCalibActive);
  output->longitudinal_calib_updated = CommandFilter_UpdateBootCalibration(&state->longitudinal,
                                                                            longitudinal_cmd,
                                                                            &longBootCalibActive);

  if (state->steering.boot_calibrated != 0U) {
    (void)CommandFilter_UpdateAdaptiveOffset(&state->steering, steering_cmd);
  }
  if (state->longitudinal.boot_calibrated != 0U) {
    output->longitudinal_calib_updated = (uint8_t)(output->longitudinal_calib_updated |
                                       CommandFilter_UpdateAdaptiveOffset(&state->longitudinal, longitudinal_cmd));
  }

  output->steering_offset = state->steering.offset;
  output->longitudinal_offset = state->longitudinal.offset;
  output->steering_cmd = (int16_t)(steering_cmd - state->steering.offset);
  output->longitudinal_cmd = (int16_t)(longitudinal_cmd - state->longitudinal.offset);

  output->longitudinal_calib_active = (uint8_t)(longBootCalibActive || (state->longitudinal.learning_zone != 0U));
  output->longitudinal_calib_locked = state->longitudinal.locked;
  output->longitudinal_calib_inhibit_torque = (uint8_t)(longBootCalibActive || steeringBootCalibActive);
}
