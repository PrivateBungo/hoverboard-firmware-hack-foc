#include "input_supervisor.h"

void InputSupervisor_Init(InputSupervisorState *state) {
  if (state == 0) {
    return;
  }
  state->timeout_latched = 0U;
}

void InputSupervisor_Update(InputSupervisorState *state, uint8_t timeout_detected) {
  if (state == 0) {
    return;
  }

  if (timeout_detected != 0U) {
    state->timeout_latched = 1U;
  }
}
