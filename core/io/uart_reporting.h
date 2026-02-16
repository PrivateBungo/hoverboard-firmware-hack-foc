#ifndef UART_REPORTING_H
#define UART_REPORTING_H

#include <stdint.h>

typedef struct {
  uint32_t frame_counter;
} UartReportingState;

void UartReporting_Init(UartReportingState *state);
void UartReporting_OnFrame(UartReportingState *state);

#endif /* UART_REPORTING_H */
