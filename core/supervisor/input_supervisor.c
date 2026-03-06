#include "input_supervisor.h"

void InputSupervisor_Init(InputSupervisorState *state) {
  if (state == 0) {
    return;
  }

  state->timeout_latched = 0U;
  state->timeout_adc = 0U;
  state->timeout_serial = 0U;
  state->timeout_general = 0U;
}

void InputSupervisor_Update(InputSupervisorState *state,
                            uint8_t timeout_adc,
                            uint8_t timeout_serial,
                            uint8_t timeout_general) {
  if (state == 0) {
    return;
  }

  state->timeout_adc = (timeout_adc != 0U) ? 1U : 0U;
  state->timeout_serial = (timeout_serial != 0U) ? 1U : 0U;
  state->timeout_general = (timeout_general != 0U) ? 1U : 0U;

  if ((state->timeout_adc != 0U) || (state->timeout_serial != 0U) || (state->timeout_general != 0U)) {
    state->timeout_latched = 1U;
  }
}

uint8_t InputSupervisor_AnyTimeout(const InputSupervisorState *state) {
  if (state == 0) {
    return 0U;
  }

  return (uint8_t)((state->timeout_adc != 0U) ||
                   (state->timeout_serial != 0U) ||
                   (state->timeout_general != 0U));
}
