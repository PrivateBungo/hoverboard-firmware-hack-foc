#ifndef STALL_PROTECTION_H
#define STALL_PROTECTION_H

#include <stdint.h>

typedef struct {
  uint16_t stall_ms;
  int16_t limited_cmd;
} StallProtectionState;

void StallProtection_Reset(StallProtectionState *state);
int16_t StallProtection_Update(StallProtectionState *state,
                               int16_t requested_cmd, int16_t measured_speed,
                               uint16_t dt_ms);

#endif
