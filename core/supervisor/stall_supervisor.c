#include "stall_supervisor.h"

#include "util.h"

#define STALL_STATE_NORMAL    (0U)
#define STALL_STATE_RAMP      (1U)
#define STALL_STATE_COOLDOWN  (2U)

#define STALL_RAMP_MS             (150U)
#define STALL_DETECT_MS           (120U)
#define STALL_RECOVERY_DELAY_MS (10000U)
#define STALL_NEUTRAL_DEADBAND    (300)
#define STALL_NEUTRAL_CLEAR_MS    (300U)
#define STALL_CMD_TRIGGER         (350)
#define STALL_SPEED_TRIGGER_RPM   (40)
#define STALL_CURRENT_TRIGGER     (350)

static uint8_t StallSupervisor_IsNeutralCommand(int16_t cmdLeft, int16_t cmdRight) {
  return (uint8_t)((ABS(cmdLeft) <= STALL_NEUTRAL_DEADBAND) &&
                   (ABS(cmdRight) <= STALL_NEUTRAL_DEADBAND));
}

static uint8_t StallSupervisor_StallCondition(int16_t reqLeft,
                                              int16_t reqRight,
                                              int16_t speedLeft,
                                              int16_t speedRight,
                                              int16_t curLeftDc,
                                              int16_t curRightDc) {
  const uint8_t lowSpeed = (uint8_t)((ABS(speedLeft) <= STALL_SPEED_TRIGGER_RPM) ||
                                     (ABS(speedRight) <= STALL_SPEED_TRIGGER_RPM));
  const uint8_t highCurrent = (uint8_t)((ABS(curLeftDc) >= STALL_CURRENT_TRIGGER) ||
                                        (ABS(curRightDc) >= STALL_CURRENT_TRIGGER));
  const uint8_t highDemand = (uint8_t)((ABS(reqLeft) >= STALL_CMD_TRIGGER) ||
                                       (ABS(reqRight) >= STALL_CMD_TRIGGER));

  return (uint8_t)(lowSpeed && (highCurrent || highDemand));
}

void StallSupervisor_Init(StallSupervisorState *state) {
  if (state == 0) {
    return;
  }

  state->state = STALL_STATE_NORMAL;
  state->stateSinceMs = 0U;
  state->detectSinceMs = 0U;
  state->neutralSinceMs = 0U;
  state->rampStartLeft = 0;
  state->rampStartRight = 0;
}

void StallSupervisor_Update(StallSupervisorState *state,
                            uint32_t tickNowMs,
                            int16_t reqLeft,
                            int16_t reqRight,
                            int16_t speedLeft,
                            int16_t speedRight,
                            int16_t curLeftDc,
                            int16_t curRightDc,
                            int16_t *outLeft,
                            int16_t *outRight,
                            uint8_t *driveEnableOut) {
  uint8_t stallCondition;

  if ((state == 0) || (outLeft == 0) || (outRight == 0) || (driveEnableOut == 0)) {
    return;
  }

  stallCondition = StallSupervisor_StallCondition(reqLeft, reqRight, speedLeft, speedRight, curLeftDc, curRightDc);

  *outLeft = reqLeft;
  *outRight = reqRight;
  *driveEnableOut = 1U;

  switch (state->state) {
    case STALL_STATE_NORMAL:
      if (stallCondition != 0U) {
        if (state->detectSinceMs == 0U) {
          state->detectSinceMs = tickNowMs;
        } else if ((tickNowMs - state->detectSinceMs) >= STALL_DETECT_MS) {
          state->state = STALL_STATE_RAMP;
          state->stateSinceMs = tickNowMs;
          state->rampStartLeft = reqLeft;
          state->rampStartRight = reqRight;
        }
      } else {
        state->detectSinceMs = 0U;
      }
      break;

    case STALL_STATE_RAMP: {
      const uint32_t elapsedMs = tickNowMs - state->stateSinceMs;

      if (elapsedMs < STALL_RAMP_MS) {
        const int32_t remainingMs = (int32_t)(STALL_RAMP_MS - elapsedMs);
        *outLeft = (int16_t)((((int32_t)state->rampStartLeft) * remainingMs) / (int32_t)STALL_RAMP_MS);
        *outRight = (int16_t)((((int32_t)state->rampStartRight) * remainingMs) / (int32_t)STALL_RAMP_MS);
      } else {
        *outLeft = 0;
        *outRight = 0;

        if (stallCondition != 0U) {
          state->state = STALL_STATE_COOLDOWN;
          state->stateSinceMs = tickNowMs;
          state->neutralSinceMs = 0U;
          *driveEnableOut = 0U;
        } else {
          state->state = STALL_STATE_NORMAL;
          state->stateSinceMs = tickNowMs;
          state->detectSinceMs = 0U;
        }
      }
      break;
    }

    case STALL_STATE_COOLDOWN: {
      const uint8_t neutralCommand = StallSupervisor_IsNeutralCommand(reqLeft, reqRight);

      *outLeft = 0;
      *outRight = 0;
      *driveEnableOut = 0U;

      if (neutralCommand != 0U) {
        if (state->neutralSinceMs == 0U) {
          state->neutralSinceMs = tickNowMs;
        }
      } else {
        state->neutralSinceMs = 0U;
      }

      if ((tickNowMs - state->stateSinceMs) >= STALL_RECOVERY_DELAY_MS &&
          (state->neutralSinceMs != 0U) &&
          ((tickNowMs - state->neutralSinceMs) >= STALL_NEUTRAL_CLEAR_MS)) {
        state->state = STALL_STATE_NORMAL;
        state->stateSinceMs = tickNowMs;
        state->detectSinceMs = 0U;
        state->neutralSinceMs = 0U;
      }
      break;
    }

    default:
      state->state = STALL_STATE_NORMAL;
      state->stateSinceMs = tickNowMs;
      state->detectSinceMs = 0U;
      state->neutralSinceMs = 0U;
      break;
  }
}

uint8_t StallSupervisor_IsActive(const StallSupervisorState *state) {
  if (state == 0) {
    return 0U;
  }

  return (uint8_t)(state->state != STALL_STATE_NORMAL);
}
