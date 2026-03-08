#include "drive_control.h"
#include "config.h"
#include "defines.h"

static int16_t clamp_signed(int16_t x, int16_t absLim) {
  if (x > absLim) {
    return absLim;
  }
  if (x < -absLim) {
    return (int16_t)-absLim;
  }
  return x;
}

int16_t DriveControl_ApplyStallDecay(int16_t cmd, int16_t speedRpm, uint8_t ctrlModReq, DriveControlStallDecayState *state) {
  const uint8_t modeEnabled =
      ((STALL_DECAY_IN_TRQ_MODE && (ctrlModReq == TRQ_MODE)) ||
       (STALL_DECAY_IN_VLT_MODE && (ctrlModReq == VLT_MODE)));

  const uint8_t activeCond = modeEnabled &&
                             (ABS(speedRpm) <= STALL_DECAY_SPEED_RPM) &&
                             (ABS(cmd) >= STALL_DECAY_CMD_TRIGGER);

  state->stallCondition = activeCond;

  if (!activeCond) {
    state->stallTimerMs = 0;
    state->stallActive = 0;
    return cmd;
  }

  const uint16_t preemptMs = MAX(STALL_DECAY_PREEMPT_MS, 1);
  const uint16_t totalMs = MAX(STALL_DECAY_TIME_MS, preemptMs + 1);
  const uint16_t loopMs = (DELAY_IN_MAIN_LOOP + 1);

  if (state->stallTimerMs < totalMs) {
    uint32_t next = (uint32_t)state->stallTimerMs + loopMs;
    state->stallTimerMs = (uint16_t)MIN(next, totalMs);
  }

  state->stallActive = (state->stallTimerMs >= preemptMs);

  int16_t cmdMax;
  if (state->stallTimerMs <= preemptMs) {
    const int32_t span = (1000 - STALL_DECAY_CMD_PREEMPT);
    cmdMax = (int16_t)(1000 - (span * state->stallTimerMs) / preemptMs);
  } else if (state->stallTimerMs < totalMs) {
    const uint16_t t2 = (uint16_t)(state->stallTimerMs - preemptMs);
    const uint16_t d2 = (uint16_t)(totalMs - preemptMs);
    const int32_t span = (STALL_DECAY_CMD_PREEMPT - STALL_DECAY_CMD_FLOOR);
    cmdMax = (int16_t)(STALL_DECAY_CMD_PREEMPT - (span * t2) / d2);
  } else {
    cmdMax = STALL_DECAY_CMD_FLOOR;
  }

  return clamp_signed(cmd, cmdMax);
}
