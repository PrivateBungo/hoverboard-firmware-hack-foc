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

uint8_t ModeSupervisor_IsStallDecayActive(const ModeSupervisorState *state,
                                          uint8_t stall_decay_trq_enabled,
                                          uint8_t trq_mode,
                                          uint8_t stall_decay_vlt_enabled,
                                          uint8_t vlt_mode) {
  uint8_t selected_mode = 0U;

  if (state != 0) {
    selected_mode = state->selected_mode;
  }

  return (uint8_t)(((stall_decay_trq_enabled != 0U) && (selected_mode == trq_mode)) ||
                   ((stall_decay_vlt_enabled != 0U) && (selected_mode == vlt_mode)));
}
