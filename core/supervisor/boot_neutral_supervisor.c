#include "boot_neutral_supervisor.h"

#define BOOT_NEUTRAL_OBS_MS     2000U
#define BOOT_NEUTRAL_ABORT_MAG   300
#define BOOT_NEUTRAL_ABS(a)      (((a) < 0) ? (-(a)) : (a))

void BootNeutralSupervisor_Init(BootNeutralSupervisorState *state, uint32_t nowMs) {
  if (state == 0) {
    return;
  }

  state->startMs = nowMs;
  state->phase = BOOT_NEUTRAL_STATE_OBSERVE;
  state->rcPresentAllWindow = 1U;
  state->abortAllWindow = 0U;
  state->sumL = 0;
  state->sumR = 0;
  state->sampleCount = 0U;
  state->neutralL = 0;
  state->neutralR = 0;
  state->observeMaxAbsL = 0;
  state->observeMaxAbsR = 0;
  state->learningApplied = 0U;
}

void BootNeutralSupervisor_Update(BootNeutralSupervisorState *state,
                                  uint32_t nowMs,
                                  uint8_t rcPresent,
                                  int16_t cmdL_filt,
                                  int16_t cmdR_filt,
                                  int16_t *cmdL_adj,
                                  int16_t *cmdR_adj,
                                  uint8_t *forceZeroPwm) {
  uint32_t elapsedMs;

  if ((state == 0) || (cmdL_adj == 0) || (cmdR_adj == 0) || (forceZeroPwm == 0)) {
    return;
  }

  elapsedMs = nowMs - state->startMs;

  *cmdL_adj = cmdL_filt;
  *cmdR_adj = cmdR_filt;
  *forceZeroPwm = 0U;

  if (state->phase == BOOT_NEUTRAL_STATE_OBSERVE) {
    state->rcPresentAllWindow = (uint8_t)(state->rcPresentAllWindow && (rcPresent != 0U));

    {
      int16_t absL = BOOT_NEUTRAL_ABS(cmdL_filt);
      int16_t absR = BOOT_NEUTRAL_ABS(cmdR_filt);

      if (absL > state->observeMaxAbsL) {
        state->observeMaxAbsL = absL;
      }
      if (absR > state->observeMaxAbsR) {
        state->observeMaxAbsR = absR;
      }

      if ((absL >= BOOT_NEUTRAL_ABORT_MAG) || (absR >= BOOT_NEUTRAL_ABORT_MAG)) {
        state->abortAllWindow = 1U;
      }
    }

    if (rcPresent != 0U) {
      state->sumL += cmdL_filt;
      state->sumR += cmdR_filt;
      state->sampleCount++;
    }

    if (elapsedMs >= BOOT_NEUTRAL_OBS_MS) {
      state->phase = BOOT_NEUTRAL_STATE_APPLY;
    }
  }

  if (state->phase == BOOT_NEUTRAL_STATE_APPLY) {
    if ((state->rcPresentAllWindow != 0U) && (state->abortAllWindow == 0U) && (state->sampleCount > 0U)) {
      state->neutralL = (int16_t)(state->sumL / (int32_t)state->sampleCount);
      state->neutralR = (int16_t)(state->sumR / (int32_t)state->sampleCount);
      state->learningApplied = 1U;
    } else {
      state->neutralL = 0;
      state->neutralR = 0;
      state->learningApplied = 0U;
    }

    state->phase = BOOT_NEUTRAL_STATE_RUN;
  }

  if (state->phase == BOOT_NEUTRAL_STATE_RUN) {
    *cmdL_adj = (int16_t)(cmdL_filt - state->neutralL);
    *cmdR_adj = (int16_t)(cmdR_filt - state->neutralR);
  } else {
    *forceZeroPwm = 1U;
  }
}

uint32_t BootNeutralSupervisor_ElapsedMs(const BootNeutralSupervisorState *state, uint32_t nowMs) {
  if (state == 0) {
    return 0U;
  }

  return nowMs - state->startMs;
}

BootNeutralSupervisorPhase BootNeutralSupervisor_GetPhase(const BootNeutralSupervisorState *state) {
  if (state == 0) {
    return BOOT_NEUTRAL_STATE_OBSERVE;
  }

  return state->phase;
}

uint8_t BootNeutralSupervisor_GetAbortFlag(const BootNeutralSupervisorState *state) {
  if (state == 0) {
    return 0U;
  }

  return state->abortAllWindow;
}

int16_t BootNeutralSupervisor_GetNeutralL(const BootNeutralSupervisorState *state) {
  if (state == 0) {
    return 0;
  }

  return state->neutralL;
}

int16_t BootNeutralSupervisor_GetNeutralR(const BootNeutralSupervisorState *state) {
  if (state == 0) {
    return 0;
  }

  return state->neutralR;
}

uint8_t BootNeutralSupervisor_GetRcPresentAllWindow(const BootNeutralSupervisorState *state) {
  if (state == 0) {
    return 0U;
  }

  return state->rcPresentAllWindow;
}

uint32_t BootNeutralSupervisor_GetSampleCount(const BootNeutralSupervisorState *state) {
  if (state == 0) {
    return 0U;
  }

  return state->sampleCount;
}

int32_t BootNeutralSupervisor_GetSumL(const BootNeutralSupervisorState *state) {
  if (state == 0) {
    return 0;
  }

  return state->sumL;
}

int32_t BootNeutralSupervisor_GetSumR(const BootNeutralSupervisorState *state) {
  if (state == 0) {
    return 0;
  }

  return state->sumR;
}

int16_t BootNeutralSupervisor_GetObserveMaxAbsL(const BootNeutralSupervisorState *state) {
  if (state == 0) {
    return 0;
  }

  return state->observeMaxAbsL;
}

int16_t BootNeutralSupervisor_GetObserveMaxAbsR(const BootNeutralSupervisorState *state) {
  if (state == 0) {
    return 0;
  }

  return state->observeMaxAbsR;
}

uint8_t BootNeutralSupervisor_WasLearningApplied(const BootNeutralSupervisorState *state) {
  if (state == 0) {
    return 0U;
  }

  return state->learningApplied;
}
