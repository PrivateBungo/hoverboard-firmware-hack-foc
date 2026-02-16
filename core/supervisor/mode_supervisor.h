#ifndef MODE_SUPERVISOR_H
#define MODE_SUPERVISOR_H

#include <stdint.h>

typedef struct {
  uint8_t selected_mode;
} ModeSupervisorState;

void ModeSupervisor_Init(ModeSupervisorState *state, uint8_t default_mode);
uint8_t ModeSupervisor_Select(ModeSupervisorState *state, uint8_t requested_mode);

#endif /* MODE_SUPERVISOR_H */
