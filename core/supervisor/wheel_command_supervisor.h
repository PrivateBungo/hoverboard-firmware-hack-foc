#ifndef WHEEL_COMMAND_SUPERVISOR_H
#define WHEEL_COMMAND_SUPERVISOR_H

#include <stdint.h>

typedef struct {
  int32_t leftFiltFixdt;
  int32_t rightFiltFixdt;
  uint8_t leftDriveEnabled;
  uint8_t rightDriveEnabled;
} WheelCommandSupervisorState;

void WheelCommandSupervisor_Init(WheelCommandSupervisorState *state);

void WheelCommandSupervisor_Update(WheelCommandSupervisorState *state,
                                   int16_t rawLeft,
                                   int16_t rawRight,
                                   int16_t *outLeft,
                                   int16_t *outRight);

#endif /* WHEEL_COMMAND_SUPERVISOR_H */
