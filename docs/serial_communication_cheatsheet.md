# Serial communication cheat sheet

This is a practical quick reference for this firmware's UART interfaces.

## 1) Physical ports and safety

- **USART2 = left sideboard cable (long cable)**, **NOT 5V tolerant**.
- **USART3 = right sideboard cable (short cable)**, **5V tolerant**.
- Both ports are configured as **115200, 8N1** by default.

## 2) Compile-time modes (pick what you need)

In `Inc/config.h`, each UART can be used for different roles, but some combinations are mutually exclusive:

- `CONTROL_SERIAL_USART2/3` → binary command input frames.
- `FEEDBACK_SERIAL_USART2/3` → binary telemetry output frames.
- `DEBUG_SERIAL_USART2/3` → text debug output (`printf`) and optional text command parser.
- `DEBUG_SERIAL_PROTOCOL` (requires DEBUG_SERIAL_USARTx) → CLI-style `$GET/$SET/...` command parser.

## 3) Binary control frame (host -> board)

When `CONTROL_SERIAL_USARTx` is enabled (non-iBUS):

```c
typedef struct{
  uint16_t start;     // 0xABCD
  int16_t  steer;     // typically -1000..1000
  int16_t  speed;     // typically -1000..1000
  uint16_t checksum;  // start ^ steer ^ speed
} SerialCommand;
```

- Start word: `0xABCD`.
- Checksum must match or frame is ignored.
- This is the same layout used by `Arduino/hoverserial/hoverserial.ino`.

## 4) Binary feedback frame (board -> host)

When `FEEDBACK_SERIAL_USARTx` is enabled:

```c
typedef struct{
  uint16_t start;       // 0xABCD
  int16_t  cmd1;
  int16_t  cmd2;
  int16_t  speedR_meas;
  int16_t  speedL_meas;
  int16_t  batVoltage;
  int16_t  boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;    // XOR of all previous fields
} SerialFeedback;
```

- Sent periodically from main loop when feedback mode is enabled.
- The Arduino example also contains parser code for this frame.

## 5) Text debug stream (board -> terminal)

When `DEBUG_SERIAL_USARTx` is enabled, `printf` is routed to that UART.

Current periodic line includes:

- `in1`, `in2`: raw selected input channels.
- `cmdL`, `cmdR`: mixed wheel commands (internal command domain, usually `-1000..1000`).
- `ErrL`, `ErrR`: current `z_errCode` bytes for left/right controller.
- battery and board temperature values.

Additionally, whenever an error code changes, an immediate line is emitted:

- `MotorErr L:<code>[b0:<0|1> b1:<0|1> b2:<0|1>] R:<...>`

This helps catch brief stall/shutdown events even with slower periodic prints.

## 6) `cmdL` / `cmdR` interpretation

`cmdL` and `cmdR` are wheel-level command values generated after input filtering/mixing.
They are **not a direct percent type**; treat them as normalized internal commands in roughly `[-1000, 1000]`, then mapped to left/right PWM sign/direction.

## 7) `z_errCode` quick decode

`z_errCode` is a bit-composed diagnostic code. In current firmware logic:

- **bit0** set when one diagnostic condition is active (`Sum == 0`).
- **bit1** set when another diagnostic condition is active (`Sum == 7`).
- **bit2** set for target-vs-standstill mismatch condition (enabled motor, near-standstill speed, but large commanded input).

If either side has non-zero `z_errCode`, motor enable is dropped and fault beeps are triggered.

## 8) Text command protocol (`DEBUG_SERIAL_PROTOCOL`)

If enabled, send lines terminated by `\n` or `\r`, prefixed with `$`.

Common commands:

- `$HELP` → print commands/parameters/variables help.
- `$GET` → print all parameter definitions.
- `$GET N_MOT_MAX` → print one parameter.
- `$SET N_MOT_MAX 900` → set parameter.
- `$WATCH CMDL` / `$WATCH CMDR` / `$WATCH SPDL` / `$WATCH SPDR` → toggle watch items.
- `$SAVE` → persist parameters to EEPROM.

Parser notes:

- Start of line must be `$`.
- End of line required (`\n` or `\r`).
- `SET` works on **parameters**, not variables.

## 9) Fast sanity checklist

1. Confirm compile flags in `Inc/config.h` match intended UART role.
2. Confirm host serial settings = `115200 8N1`.
3. Verify electrical level compatibility (especially USART2).
4. For binary mode, verify start frame `0xABCD` and checksum.
5. For debug protocol, verify each command starts with `$` and ends with newline.
