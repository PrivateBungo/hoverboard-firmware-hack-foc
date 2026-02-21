#ifndef UART_REPORTING_H
#define UART_REPORTING_H

#include <stdint.h>

typedef struct {
  uint32_t frame_counter;
} UartReportingState;

typedef struct {
  uint16_t start;
  int16_t cmd1;
  int16_t cmd2;
  int16_t speedR_meas;
  int16_t speedL_meas;
  int16_t batVoltage;
  int16_t boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} UartReportingFrame;

void UartReporting_Init(UartReportingState *state);
void UartReporting_OnFrame(UartReportingState *state);
void UartReporting_PrepareFrame(UartReportingFrame *frame,
                                uint16_t start,
                                int16_t cmd1,
                                int16_t cmd2,
                                int16_t speed_r_meas,
                                int16_t speed_l_meas,
                                int16_t bat_voltage,
                                int16_t board_temp,
                                uint16_t cmd_led);

#endif /* UART_REPORTING_H */
