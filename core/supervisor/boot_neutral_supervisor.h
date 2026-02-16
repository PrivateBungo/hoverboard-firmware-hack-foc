#ifndef BOOT_NEUTRAL_SUPERVISOR_H
#define BOOT_NEUTRAL_SUPERVISOR_H

#include <stdint.h>

typedef enum {
  BOOT_NEUTRAL_STATE_OBSERVE = 0,
  BOOT_NEUTRAL_STATE_APPLY,
  BOOT_NEUTRAL_STATE_RUN
} BootNeutralState;

typedef struct {
  BootNeutralState state;
  uint32_t boot_t0;
  uint8_t rc_present_all_window;
  uint8_t stable_neutral_all_window;
  int32_t sumLeft;
  int32_t sumRight;
  uint16_t sampleCount;
  int16_t neutralLeft;
  int16_t neutralRight;
  uint8_t neutral_active;
  uint32_t bootDbgLastMs;
} BootNeutralSupervisorState;

void BootNeutralSupervisor_Init(BootNeutralSupervisorState *state, uint32_t nowMs);

uint8_t BootNeutralSupervisor_IsRcInputSignalPresent(uint8_t inIdx,
                                                     uint8_t timeoutFlgGen,
                                                     uint8_t timeoutFlgSerial);

void BootNeutralSupervisor_Process(BootNeutralSupervisorState *state,
                                   uint32_t nowMs,
                                   uint8_t rcPresent,
                                   uint8_t timeoutFlgGen,
                                   uint8_t timeoutFlgSerial,
                                   uint8_t ctrlModReq,
                                   uint8_t enable,
                                   int16_t cmdL_filt,
                                   int16_t cmdR_filt,
                                   int16_t *cmdL_adj,
                                   int16_t *cmdR_adj,
                                   uint8_t *forcePwmZero);

#endif /* BOOT_NEUTRAL_SUPERVISOR_H */
