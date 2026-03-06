#ifndef FOC_ADAPTER_H
#define FOC_ADAPTER_H

#include <stdint.h>
#include "BLDC_controller.h"

typedef enum {
  FOC_ADAPTER_MOTOR_LEFT = 0,
  FOC_ADAPTER_MOTOR_RIGHT = 1
} FocAdapterMotor;

typedef struct {
  uint8_t motorEnable;
  uint8_t controlModeRequest;
  int16_t inputTarget;
  uint8_t hallA;
  uint8_t hallB;
  uint8_t hallC;
  int16_t phaseCurrentAB;
  int16_t phaseCurrentBC;
  int16_t dcLinkCurrent;
} FocAdapterInputFrame;

void FocAdapter_SetInputFrame(FocAdapterMotor motor, const FocAdapterInputFrame *frame);
void FocAdapter_Step(FocAdapterMotor motor);

const ExtY *FocAdapter_GetOutput(FocAdapterMotor motor);
ExtU *FocAdapter_GetInput(FocAdapterMotor motor);
P *FocAdapter_GetParams(FocAdapterMotor motor);

uint8_t FocAdapter_GetErrorCode(FocAdapterMotor motor);
int16_t FocAdapter_GetMotorSpeed(FocAdapterMotor motor);
int16_t FocAdapter_GetDcLinkCurrent(FocAdapterMotor motor);

void FocAdapter_ClearErrorBits(uint8_t errorMask);

#endif /* FOC_ADAPTER_H */
