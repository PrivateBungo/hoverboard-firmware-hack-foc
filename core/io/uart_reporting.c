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
