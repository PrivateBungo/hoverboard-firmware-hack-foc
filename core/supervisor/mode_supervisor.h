#ifndef MODE_SUPERVISOR_H
#define MODE_SUPERVISOR_H

#include <stdint.h>

typedef struct {
  uint8_t selected_mode;
} ModeSupervisorState;

void ModeSupervisor_Init(ModeSupervisorState *state, uint8_t default_mode);
uint8_t ModeSupervisor_Select(ModeSupervisorState *state, uint8_t requested_mode);
uint8_t ModeSupervisor_IsStallDecayActive(const ModeSupervisorState *state,
                                          uint8_t stall_decay_trq_enabled,
                                          uint8_t trq_mode,
                                          uint8_t stall_decay_vlt_enabled,
                                          uint8_t vlt_mode);

#endif /* MODE_SUPERVISOR_H */
