#include "wheel_command_supervisor.h"

#include "config.h"

static int16_t WheelCommandSupervisor_LpfStep(int16_t input, int32_t *stateFixdt) {
  int32_t inputFixdt;
  int32_t delta;

  if (WHEEL_CMD_FILTER_COEF >= 65535) {
    *stateFixdt = ((int32_t)input) << 16;
    return input;
  }

  inputFixdt = ((int32_t)input) << 16;
  delta = inputFixdt - *stateFixdt;
  *stateFixdt += (int32_t)(((int64_t)delta * WHEEL_CMD_FILTER_COEF) >> 16);

  return (int16_t)(*stateFixdt >> 16);
}

void WheelCommandSupervisor_Init(WheelCommandSupervisorState *state) {
  if (state == 0) {
    return;
  }

  state->leftFiltFixdt = 0;
  state->rightFiltFixdt = 0;
}

void WheelCommandSupervisor_Update(WheelCommandSupervisorState *state,
                                   int16_t rawLeft,
                                   int16_t rawRight,
                                   int16_t *outLeft,
                                   int16_t *outRight) {
  int16_t filteredLeft;
  int16_t filteredRight;

  if ((state == 0) || (outLeft == 0) || (outRight == 0)) {
    return;
  }

  filteredLeft = WheelCommandSupervisor_LpfStep(rawLeft, &state->leftFiltFixdt);
  filteredRight = WheelCommandSupervisor_LpfStep(rawRight, &state->rightFiltFixdt);

  *outLeft = filteredLeft;
  *outRight = filteredRight;
}
