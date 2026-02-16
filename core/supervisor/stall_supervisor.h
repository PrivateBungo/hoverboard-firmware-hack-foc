#ifndef STALL_SUPERVISOR_H
#define STALL_SUPERVISOR_H

#include <stdint.h>

typedef struct {
  uint8_t state;
  uint32_t stateSinceMs;
  uint32_t detectSinceMs;
  uint32_t neutralSinceMs;
  int16_t rampStartLeft;
  int16_t rampStartRight;
} StallSupervisorState;

void StallSupervisor_Init(StallSupervisorState *state);

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
                            uint8_t *driveEnableOut);

uint8_t StallSupervisor_IsActive(const StallSupervisorState *state);

#endif /* STALL_SUPERVISOR_H */
