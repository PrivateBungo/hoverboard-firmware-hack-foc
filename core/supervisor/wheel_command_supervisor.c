#include "wheel_command_supervisor.h"

#include "config.h"

#define WHEEL_CMD_ABS(a) (((a) < 0) ? (-(a)) : (a))

static int16_t WheelCommandSupervisor_LpfStep(int16_t input, int32_t *stateFixdt) {
  int32_t inputFixdt;
  int32_t delta;

  inputFixdt = ((int32_t)input) << 16;
  delta = inputFixdt - *stateFixdt;
  *stateFixdt += (int32_t)(((int64_t)delta * WHEEL_CMD_FILTER_COEF) >> 16);

  return (int16_t)(*stateFixdt >> 16);
}

static int16_t WheelCommandSupervisor_ApplyHysteresis(int16_t filtered,
                                                      uint8_t *driveEnabled) {
  int16_t absFiltered;

  absFiltered = WHEEL_CMD_ABS(filtered);

  if (*driveEnabled == 0U) {
    if (absFiltered >= WHEEL_CMD_HYST_ON) {
      *driveEnabled = 1U;
    }
  } else {
    if ((absFiltered <= WHEEL_CMD_DEADBAND) || (absFiltered <= WHEEL_CMD_HYST_OFF)) {
      *driveEnabled = 0U;
    }
  }

  if (*driveEnabled == 0U) {
    return 0;
  }

  return filtered;
}

void WheelCommandSupervisor_Init(WheelCommandSupervisorState *state) {
  if (state == 0) {
    return;
  }

  state->leftFiltFixdt = 0;
  state->rightFiltFixdt = 0;
  state->leftDriveEnabled = 0U;
  state->rightDriveEnabled = 0U;
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

  *outLeft = WheelCommandSupervisor_ApplyHysteresis(filteredLeft, &state->leftDriveEnabled);
  *outRight = WheelCommandSupervisor_ApplyHysteresis(filteredRight, &state->rightDriveEnabled);
}
