#include "stall_supervisor.h"

#include "config.h"

#define STALL_ABS(a) (((a) < 0) ? (-(a)) : (a))

#define STALL_STATE_NORMAL    (0U)
#define STALL_STATE_RAMP      (1U)
#define STALL_STATE_HOLD      (2U)

#define STALL_RAMP_MS             (150U)
#define STALL_DETECT_MS           (120U)
#define STALL_CMD_TRIGGER         (350)
#define STALL_SPEED_TRIGGER_RPM   (40)
#define STALL_CURRENT_TRIGGER     (350)

static int16_t StallSupervisor_TargetFromRequest(int16_t req) {
  if (req > STALL_DECAY_CMD_FLOOR) {
    return STALL_DECAY_CMD_FLOOR;
  }

  if (req < -STALL_DECAY_CMD_FLOOR) {
    return (int16_t)-STALL_DECAY_CMD_FLOOR;
  }

  return req;
}

static uint8_t StallSupervisor_StallCondition(int16_t reqLeft,
                                              int16_t reqRight,
                                              int16_t speedLeft,
                                              int16_t speedRight,
                                              int16_t curLeftDc,
                                              int16_t curRightDc) {
  const uint8_t lowSpeed = (uint8_t)((STALL_ABS(speedLeft) <= STALL_SPEED_TRIGGER_RPM) ||
                                     (STALL_ABS(speedRight) <= STALL_SPEED_TRIGGER_RPM));
  const uint8_t highCurrent = (uint8_t)((STALL_ABS(curLeftDc) >= STALL_CURRENT_TRIGGER) ||
                                        (STALL_ABS(curRightDc) >= STALL_CURRENT_TRIGGER));
  const uint8_t highDemand = (uint8_t)((STALL_ABS(reqLeft) >= STALL_CMD_TRIGGER) ||
                                       (STALL_ABS(reqRight) >= STALL_CMD_TRIGGER));

  return (uint8_t)(lowSpeed && (highCurrent || highDemand));
}

void StallSupervisor_Init(StallSupervisorState *state) {
  if (state == 0) {
    return;
  }

  state->state = STALL_STATE_NORMAL;
  state->stateSinceMs = 0U;
  state->detectSinceMs = 0U;
  state->rampStartLeft = 0;
  state->rampStartRight = 0;
  state->rampTargetLeft = 0;
  state->rampTargetRight = 0;
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
          state->rampTargetLeft = StallSupervisor_TargetFromRequest(reqLeft);
          state->rampTargetRight = StallSupervisor_TargetFromRequest(reqRight);
        }
      } else {
        state->detectSinceMs = 0U;
      }
      break;

    case STALL_STATE_RAMP: {
      const uint32_t elapsedMs = tickNowMs - state->stateSinceMs;

      if (elapsedMs < STALL_RAMP_MS) {
        const int32_t elapsed = (int32_t)elapsedMs;
        const int32_t spanLeft = (int32_t)state->rampStartLeft - (int32_t)state->rampTargetLeft;
        const int32_t spanRight = (int32_t)state->rampStartRight - (int32_t)state->rampTargetRight;
        *outLeft = (int16_t)((int32_t)state->rampStartLeft - ((spanLeft * elapsed) / (int32_t)STALL_RAMP_MS));
        *outRight = (int16_t)((int32_t)state->rampStartRight - ((spanRight * elapsed) / (int32_t)STALL_RAMP_MS));
      } else {
        *outLeft = state->rampTargetLeft;
        *outRight = state->rampTargetRight;
        state->state = STALL_STATE_HOLD;
        state->stateSinceMs = tickNowMs;
      }
      break;
    }

    case STALL_STATE_HOLD:
      if (stallCondition != 0U) {
        *outLeft = StallSupervisor_TargetFromRequest(reqLeft);
        *outRight = StallSupervisor_TargetFromRequest(reqRight);
      } else {
        state->state = STALL_STATE_NORMAL;
        state->stateSinceMs = tickNowMs;
        state->detectSinceMs = 0U;
      }
      break;

    default:
      state->state = STALL_STATE_NORMAL;
      state->stateSinceMs = tickNowMs;
      state->detectSinceMs = 0U;
      break;
  }
}

uint8_t StallSupervisor_IsActive(const StallSupervisorState *state) {
  if (state == 0) {
    return 0U;
  }

  return (uint8_t)(state->state != STALL_STATE_NORMAL);
}
