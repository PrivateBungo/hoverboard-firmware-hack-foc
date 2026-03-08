#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include <stdint.h>

typedef struct {
  uint16_t stallTimerMs;
  uint8_t  stallCondition;
  uint8_t  stallActive;
} DriveControlStallDecayState;

int16_t DriveControl_ApplyStallDecay(int16_t cmd, int16_t speedRpm, uint8_t ctrlModReq, DriveControlStallDecayState *state);

#endif
