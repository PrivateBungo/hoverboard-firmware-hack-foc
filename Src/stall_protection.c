#include "stall_protection.h"
#include "config.h"
#include "defines.h"

#ifndef STALL_PROTECTION_CMD_THRESHOLD
#define STALL_PROTECTION_CMD_THRESHOLD 650
#endif

#ifndef STALL_PROTECTION_SPEED_THRESHOLD_RPM
#define STALL_PROTECTION_SPEED_THRESHOLD_RPM 30
#endif

#ifndef STALL_PROTECTION_GRACE_MS
#define STALL_PROTECTION_GRACE_MS 1000
#endif

#ifndef STALL_PROTECTION_RAMP_MS
#define STALL_PROTECTION_RAMP_MS 1000
#endif

#ifndef STALL_PROTECTION_FULL_CMD
#define STALL_PROTECTION_FULL_CMD 1000
#endif

#ifndef STALL_PROTECTION_SUSTAIN_CMD
#define STALL_PROTECTION_SUSTAIN_CMD 350
#endif

void StallProtection_Reset(StallProtectionState *state) {
  state->stall_ms = 0;
  state->limited_cmd = 0;
}

int16_t StallProtection_Update(StallProtectionState *state,
                               int16_t requested_cmd, int16_t measured_speed,
                               uint16_t dt_ms) {
#ifndef STALL_PROTECTION_ENABLE
  state->limited_cmd = requested_cmd;
  state->stall_ms = 0;
  return requested_cmd;
#else
  const int16_t requested_abs = ABS(requested_cmd);
  const int16_t speed_abs = ABS(measured_speed);

  if (requested_abs <= STALL_PROTECTION_CMD_THRESHOLD ||
      speed_abs >= STALL_PROTECTION_SPEED_THRESHOLD_RPM) {
    state->stall_ms = 0;
    state->limited_cmd = requested_cmd;
    return requested_cmd;
  }

  const uint16_t elapsed_ms = state->stall_ms;
  int16_t ceiling = STALL_PROTECTION_FULL_CMD;

  if (elapsed_ms > STALL_PROTECTION_GRACE_MS) {
    const uint16_t ramp_ms = elapsed_ms - STALL_PROTECTION_GRACE_MS;
    if (ramp_ms >= STALL_PROTECTION_RAMP_MS) {
      ceiling = STALL_PROTECTION_SUSTAIN_CMD;
    } else {
      const int32_t derate_span =
          STALL_PROTECTION_FULL_CMD - STALL_PROTECTION_SUSTAIN_CMD;
      ceiling = (int16_t)(STALL_PROTECTION_FULL_CMD -
                          ((derate_span * ramp_ms) / STALL_PROTECTION_RAMP_MS));
    }
  }

  if (requested_abs > ceiling) {
    state->limited_cmd = (requested_cmd < 0) ? -ceiling : ceiling;
  } else {
    state->limited_cmd = requested_cmd;
  }

  if ((uint32_t)state->stall_ms + dt_ms > UINT16_MAX) {
    state->stall_ms = UINT16_MAX;
  } else {
    state->stall_ms += dt_ms;
  }

  return state->limited_cmd;
#endif
}
