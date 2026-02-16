#include "mode_supervisor.h"

void ModeSupervisor_Init(ModeSupervisorState *state, uint8_t default_mode) {
  if (state == 0) {
    return;
  }
  state->selected_mode = default_mode;
}

uint8_t ModeSupervisor_Select(ModeSupervisorState *state, uint8_t requested_mode) {
  if (state == 0) {
    return requested_mode;
  }

  state->selected_mode = requested_mode;
  return state->selected_mode;
}
