#ifndef BOOT_NEUTRAL_SUPERVISOR_H
#define BOOT_NEUTRAL_SUPERVISOR_H

#include <stdint.h>

typedef enum {
  BOOT_NEUTRAL_STATE_OBSERVE = 0,
  BOOT_NEUTRAL_STATE_APPLY,
  BOOT_NEUTRAL_STATE_RUN
} BootNeutralSupervisorPhase;

typedef struct {
  uint32_t startMs;
  BootNeutralSupervisorPhase phase;
  uint8_t rcPresentAllWindow;
  uint8_t abortAllWindow;
  int32_t sumL;
  int32_t sumR;
  uint32_t sampleCount;
  int16_t neutralL;
  int16_t neutralR;
  int16_t observeMaxAbsL;
  int16_t observeMaxAbsR;
  uint8_t learningApplied;
} BootNeutralSupervisorState;

void BootNeutralSupervisor_Init(BootNeutralSupervisorState *state, uint32_t nowMs);
void BootNeutralSupervisor_Update(BootNeutralSupervisorState *state,
                                  uint32_t nowMs,
                                  uint8_t rcPresent,
                                  int16_t cmdL_filt,
                                  int16_t cmdR_filt,
                                  int16_t *cmdL_adj,
                                  int16_t *cmdR_adj,
                                  uint8_t *forceZeroPwm);

uint32_t BootNeutralSupervisor_ElapsedMs(const BootNeutralSupervisorState *state, uint32_t nowMs);
BootNeutralSupervisorPhase BootNeutralSupervisor_GetPhase(const BootNeutralSupervisorState *state);
uint8_t BootNeutralSupervisor_GetAbortFlag(const BootNeutralSupervisorState *state);
int16_t BootNeutralSupervisor_GetNeutralL(const BootNeutralSupervisorState *state);
int16_t BootNeutralSupervisor_GetNeutralR(const BootNeutralSupervisorState *state);
uint8_t BootNeutralSupervisor_GetRcPresentAllWindow(const BootNeutralSupervisorState *state);
uint32_t BootNeutralSupervisor_GetSampleCount(const BootNeutralSupervisorState *state);
int32_t BootNeutralSupervisor_GetSumL(const BootNeutralSupervisorState *state);
int32_t BootNeutralSupervisor_GetSumR(const BootNeutralSupervisorState *state);
int16_t BootNeutralSupervisor_GetObserveMaxAbsL(const BootNeutralSupervisorState *state);
int16_t BootNeutralSupervisor_GetObserveMaxAbsR(const BootNeutralSupervisorState *state);
uint8_t BootNeutralSupervisor_WasLearningApplied(const BootNeutralSupervisorState *state);

#endif /* BOOT_NEUTRAL_SUPERVISOR_H */
