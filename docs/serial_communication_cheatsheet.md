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

### Mutual exclusion rules

Each USART pin-pair (TX/RX) can serve **only one** role at a time:

| Mode | Can coexist on same USART? |
|------|---------------------------|
| `CONTROL_SERIAL_USARTx` + `FEEDBACK_SERIAL_USARTx` | ✅ Yes – control (RX) and feedback (TX) share the port |
| `DEBUG_SERIAL_USARTx` + `CONTROL_SERIAL_USARTx` | ❌ No – both compete for the same RX path |
| `DEBUG_SERIAL_USARTx` + `FEEDBACK_SERIAL_USARTx` | ❌ No – both write to the same TX path |
| `SIDEBOARD_SERIAL_USARTx` + `CONTROL_SERIAL_USARTx` | ❌ No – incompatible frame formats |

### How to switch from DEBUG to CONTROL on USART3 (recommended)

1. Open `Inc/config.h` and locate `VARIANT_USART` (or whichever variant you use).
2. Comment out (or remove) `DEBUG_SERIAL_USART3`.
3. Uncomment both of these:
   ```c
   #define CONTROL_SERIAL_USART3  0   // enables binary RX command frames
   #define FEEDBACK_SERIAL_USART3     // enables binary TX telemetry frames
   ```
4. Optionally enable ACK replies (see §10 below):
   ```c
   // In the UART SETTINGS block, already present as a comment:
   #define CONTROL_SERIAL_ACK
   ```
5. Rebuild and flash.

After flashing you will **no longer see `printf` debug text** on USART3; instead the board
accepts binary `SerialCommand` frames (RX) and emits binary `SerialFeedback` frames (TX).

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

### RX data path (informational)

```
Host TX
  │  (8 bytes, binary SerialCommand)
  ▼
USART3 RX pin  ──► DMA circular buffer (rx_buffer_R, 64 bytes)
                        │
               USART IDLE-line ISR
                        │
               usart3_rx_check()   ← called from stm32f1xx_it.c USART3_IRQHandler
                        │  detects complete frame by exact byte-count match
               usart_process_command()
                        │  validates start word + XOR checksum
                        ▼
               commandR  (accepted command, module-static)
                        │
               readCommand()  ← called from main loop every 5 ms
                        │
               readInputRaw() → calcInputCmd() → mixerFcn()
                        │
               cmdL / cmdR  → FOC motor controllers
```

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

## 10) Optional ACK mechanism (`CONTROL_SERIAL_ACK`)

When `CONTROL_SERIAL_ACK` is defined (in the UART SETTINGS block of `Inc/config.h`), the
firmware replies with a lightweight **SerialAck** frame immediately after accepting each valid
`SerialCommand` frame.

### SerialAck frame (board -> host, 10 bytes)

```c
#define SERIAL_ACK_FRAME  0xCDAB   // distinct start word, never confused with SerialFeedback

typedef struct{
  uint16_t start;     // SERIAL_ACK_FRAME = 0xCDAB
  uint16_t seq;       // rolling 16-bit counter (wraps 0..65535); detects dropped ACKs
  int16_t  steer;     // echo of last accepted steer value
  int16_t  speed;     // echo of last accepted speed value
  uint16_t checksum;  // start ^ seq ^ (uint16_t)steer ^ (uint16_t)speed
} SerialAck;
```

### Design notes

- ACK is sent via **blocking `HAL_UART_Transmit`** with a 5 ms timeout from within the
  USART IDLE-line ISR (same context as `usart_process_command`).
  At 115200 baud, 10 bytes takes ≈ 0.9 ms – well within the 5 ms budget.
- ACK does **not** affect `SerialFeedback` frames; those continue to be sent by the main loop
  via DMA, independently.
- If the host is half-duplex (shared TX/RX line), it must stop transmitting for the ≈ 0.9 ms
  after sending a command to allow the ACK to arrive.
- Missed ACKs are safe – the `seq` counter lets the host detect gaps without blocking.
- Do **not** enable `CONTROL_SERIAL_ACK` together with `CONTROL_IBUS`; iBUS has its own
  protocol and does not use `SerialCommand` frames.

### Enabling ACK

In `Inc/config.h`, inside the `// ########################### UART SETTINGS ############################`
block, change:

```c
// #define CONTROL_SERIAL_ACK
```
to:
```c
#define CONTROL_SERIAL_ACK
```

## 11) Host-side Python example

See `examples/python_serial/hoverserial.py` for a complete pyserial example that:
- Sends `SerialCommand` frames.
- Reads and validates `SerialFeedback` frames.
- Optionally reads and validates `SerialAck` frames when `CONTROL_SERIAL_ACK` is enabled.

### Quick bench-test procedure (wheels off ground)

1. Flash firmware compiled with `CONTROL_SERIAL_USART3`, `FEEDBACK_SERIAL_USART3`, and
   optionally `CONTROL_SERIAL_ACK`.
2. Connect a USB-to-3.3V/5V UART adapter to the right (short) sensor cable:
   - GND ↔ GND, TX(adapter) ↔ RX(board), RX(adapter) ↔ TX(board).
   - USART3 is 5V tolerant, so a 5V adapter is fine.
3. Install pyserial: `pip install pyserial`.
4. Run the example:
   ```bash
   python examples/python_serial/hoverserial.py --port /dev/ttyUSB0
   ```
5. Expected output (with ACK enabled):
   ```
   Sent: steer=0 speed=50
   ACK  seq=1  steer=0  speed=50   ✓
   FB   cmd1=0 cmd2=50  speedR=45  speedL=45  bat=3980  temp=28  led=0
   ```
6. Gradually increase `speed` (e.g., up to 300) while the board is on a stand.
   Verify `speedR_meas` and `speedL_meas` in feedback rise proportionally.
7. Send `speed=0` to stop. Confirm both speeds return to 0.
