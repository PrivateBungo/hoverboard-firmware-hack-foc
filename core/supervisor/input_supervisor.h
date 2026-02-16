#ifndef INPUT_SUPERVISOR_H
#define INPUT_SUPERVISOR_H

#include <stdint.h>

typedef struct {
  uint8_t timeout_latched;
} InputSupervisorState;

void InputSupervisor_Init(InputSupervisorState *state);
void InputSupervisor_Update(InputSupervisorState *state, uint8_t timeout_detected);

#endif /* INPUT_SUPERVISOR_H */
