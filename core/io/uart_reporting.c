#include "uart_reporting.h"

void UartReporting_Init(UartReportingState *state) {
  if (state == 0) {
    return;
  }
  state->frame_counter = 0U;
}

void UartReporting_OnFrame(UartReportingState *state) {
  if (state == 0) {
    return;
  }
  state->frame_counter++;
}

void UartReporting_PrepareFrame(UartReportingFrame *frame,
                                uint16_t start,
                                int16_t cmd1,
                                int16_t cmd2,
                                int16_t speed_r_meas,
                                int16_t speed_l_meas,
                                int16_t bat_voltage,
                                int16_t board_temp,
                                uint16_t cmd_led) {
  if (frame == 0) {
    return;
  }

  frame->start = start;
  frame->cmd1 = cmd1;
  frame->cmd2 = cmd2;
  frame->speedR_meas = speed_r_meas;
  frame->speedL_meas = speed_l_meas;
  frame->batVoltage = bat_voltage;
  frame->boardTemp = board_temp;
  frame->cmdLed = cmd_led;
  frame->checksum = (uint16_t)(frame->start ^ frame->cmd1 ^ frame->cmd2 ^ frame->speedR_meas ^ frame->speedL_meas
                             ^ frame->batVoltage ^ frame->boardTemp ^ frame->cmdLed);
}
