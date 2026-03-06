#ifndef INPUT_SUPERVISOR_H
#define INPUT_SUPERVISOR_H

#include <stdint.h>

typedef struct {
  uint8_t timeout_latched;
  uint8_t timeout_adc;
  uint8_t timeout_serial;
  uint8_t timeout_general;
} InputSupervisorState;

void InputSupervisor_Init(InputSupervisorState *state);
void InputSupervisor_Update(InputSupervisorState *state,
                            uint8_t timeout_adc,
                            uint8_t timeout_serial,
                            uint8_t timeout_general);
uint8_t InputSupervisor_AnyTimeout(const InputSupervisorState *state);

#endif /* INPUT_SUPERVISOR_H */
