#include "foc_adapter.h"

extern RT_MODEL *const rtM_Left;
extern RT_MODEL *const rtM_Right;

extern DW rtDW_Left;
extern ExtU rtU_Left;
extern ExtY rtY_Left;
extern P rtP_Left;

extern DW rtDW_Right;
extern ExtU rtU_Right;
extern ExtY rtY_Right;
extern P rtP_Right;

static ExtU *FocAdapter_SelectInput(FocAdapterMotor motor) {
  return (motor == FOC_ADAPTER_MOTOR_LEFT) ? &rtU_Left : &rtU_Right;
}

static ExtY *FocAdapter_SelectOutput(FocAdapterMotor motor) {
  return (motor == FOC_ADAPTER_MOTOR_LEFT) ? &rtY_Left : &rtY_Right;
}

static RT_MODEL *const FocAdapter_SelectModel(FocAdapterMotor motor) {
  return (motor == FOC_ADAPTER_MOTOR_LEFT) ? rtM_Left : rtM_Right;
}

void FocAdapter_SetInputFrame(FocAdapterMotor motor, const FocAdapterInputFrame *frame) {
  ExtU *const input = FocAdapter_SelectInput(motor);

  input->b_motEna = frame->motorEnable;
  input->z_ctrlModReq = frame->controlModeRequest;
  input->r_inpTgt = frame->inputTarget;
  input->b_hallA = frame->hallA;
  input->b_hallB = frame->hallB;
  input->b_hallC = frame->hallC;
  input->i_phaAB = frame->phaseCurrentAB;
  input->i_phaBC = frame->phaseCurrentBC;
  input->i_DCLink = frame->dcLinkCurrent;
}

void FocAdapter_Step(FocAdapterMotor motor) {
  BLDC_controller_step(FocAdapter_SelectModel(motor));
}

const ExtY *FocAdapter_GetOutput(FocAdapterMotor motor) {
  return FocAdapter_SelectOutput(motor);
}

ExtU *FocAdapter_GetInput(FocAdapterMotor motor) {
  return FocAdapter_SelectInput(motor);
}

P *FocAdapter_GetParams(FocAdapterMotor motor) {
  return (motor == FOC_ADAPTER_MOTOR_LEFT) ? &rtP_Left : &rtP_Right;
}

uint8_t FocAdapter_GetErrorCode(FocAdapterMotor motor) {
  return FocAdapter_SelectOutput(motor)->z_errCode;
}

int16_t FocAdapter_GetMotorSpeed(FocAdapterMotor motor) {
  return (int16_t)FocAdapter_SelectOutput(motor)->n_mot;
}

int16_t FocAdapter_GetDcLinkCurrent(FocAdapterMotor motor) {
  return FocAdapter_SelectInput(motor)->i_DCLink;
}

void FocAdapter_ClearErrorBits(uint8_t errorMask) {
  rtDW_Left.UnitDelay_DSTATE_e &= (uint8_T)~errorMask;
  rtDW_Right.UnitDelay_DSTATE_e &= (uint8_T)~errorMask;
}
