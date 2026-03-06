# UART Architecture

This document describes the complete serial/UART architecture of this firmware —
physical ports, all compile-time roles, mutual-exclusion rules, per-variant
allocations, and how `printf` is routed. It is intended to cut through the
accumulated complexity from the upstream debug path, sideboard protocol, and the
later-added binary control+feedback path.

---

## 1. Physical ports

| Peripheral | Connector    | Baud (default) | Logic level      | Notes |
|------------|-------------|----------------|------------------|-------|
| **USART2** | **Left** sideboard cable (long) | 115200 8N1 | **3.3 V only – NOT 5 V tolerant** | PA2 = TX, PA3 = RX |
| **USART3** | **Right** sideboard cable (short) | 115200 8N1 | 5 V tolerant | PB10 = TX, PB11 = RX |

> ⚠️ The left cable (USART2) is **not** 5 V tolerant. Use a 3.3 V USB-UART adapter only.
> Never connect the red 15 V wire to your adapter.

Baud rates can be overridden per-variant by defining `USART2_BAUD` / `USART3_BAUD`
before the UART settings block in `Inc/config.h`.

---

## 2. Compile-time serial roles

Each physical UART is assigned at most **one** role per port at build time.
The roles are selected by defining one of the following macros in `Inc/config.h`:

| Macro | Direction | Wire format | Description |
|-------|-----------|-------------|-------------|
| `DEBUG_SERIAL_USART2/3` | TX only | ASCII text | Routes all `printf` calls to that UART. Human-readable. Sends periodic telemetry lines and event logs. |
| `DEBUG_SERIAL_PROTOCOL` | TX + RX | ASCII text | Adds a `$GET`/`$SET`/`$WATCH` command parser on top of `DEBUG_SERIAL_USARTx` (requires one of those to be defined first). |
| `CONTROL_SERIAL_USART2/3` | RX | Binary frames | Receives `SerialCommand` frames from the host (start `0xABCD`, steer, speed, XOR checksum). |
| `FEEDBACK_SERIAL_USART2/3` | TX | Binary frames | Transmits `UartReportingFrame` / `SerialFeedback` frames to the host every ~10 ms (DMA). |
| `SIDEBOARD_SERIAL_USART2/3` | RX | Binary frames | Receives `SerialSideboard` frames from the hacked sideboards. Mutually exclusive with `CONTROL_SERIAL_USARTx` on the same port. |

### 2.1 What `printf` actually does

`printf` is **only routed to a UART** when `DEBUG_SERIAL_USART2` **or**
`DEBUG_SERIAL_USART3` is defined. The routing is done via a custom `fputc`/`_write`
implementation in `Src/util.c`:

```c
// Src/util.c (simplified)
#if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
  int __io_putchar(int ch) {
  #if defined(DEBUG_SERIAL_USART2)
    HAL_UART_Transmit(&huart2, &ch, 1, 1000);   // ASCII byte → USART2 TX
  #elif defined(DEBUG_SERIAL_USART3)
    HAL_UART_Transmit(&huart3, &ch, 1, 1000);   // ASCII byte → USART3 TX
  #endif
    return ch;
  }
#endif
```

If **neither** `DEBUG_SERIAL_USART2` nor `DEBUG_SERIAL_USART3` is defined, the
custom `_write` is not compiled in at all and `printf` calls are no-ops (newlib
weak stub). **No text output appears on any wire.**

Conversely, if `DEBUG_SERIAL_USART2` *is* defined, **every** `printf` in the
firmware — including boot-time calibration messages, safety-fault logs, and the
high-frequency `SetpointTrace` lines — goes out on USART2 TX.

### 2.2 `CONTROL_SERIAL` vs. `FEEDBACK_SERIAL` vs. `DEBUG_SERIAL`

These roles use the **same physical wire pair** (TX + RX of a single port), so
some combinations physically conflict even though TX and RX are different pins:

- `FEEDBACK_SERIAL_USARTx` uses DMA on **TX**.
- `DEBUG_SERIAL_USARTx` uses blocking `HAL_UART_Transmit` on **TX**.
  → **Both cannot share TX** on the same port.

- `CONTROL_SERIAL_USARTx` uses DMA on **RX**.
- `DEBUG_SERIAL_USARTx` uses DMA (RX receive buffer) on **RX** for
  `DEBUG_SERIAL_PROTOCOL`.
  → Less of a problem electrically, but logically confusing to the host.

The firmware enforces the critical conflict at compile time (see §3).

---

## 3. Mutual-exclusion rules (enforced by `#error` in `Inc/config.h`)

| Combination | Error message |
|-------------|---------------|
| `DEBUG_SERIAL_USART2` + `FEEDBACK_SERIAL_USART2` | "not allowed, choose one" |
| `DEBUG_SERIAL_USART3` + `FEEDBACK_SERIAL_USART3` | "not allowed, choose one" |
| `DEBUG_SERIAL_USART2` + `DEBUG_SERIAL_USART3` | "not allowed, choose one" |
| `CONTROL_SERIAL_USART2` + `SIDEBOARD_SERIAL_USART2` | "not allowed, choose one" |
| `CONTROL_SERIAL_USART3` + `SIDEBOARD_SERIAL_USART3` | "not allowed, choose one" |
| `CONTROL_ADC` + any SERIAL_USART2 | "on the same cable" |
| `CONTROL_PPM_LEFT/RIGHT` + SERIAL on that cable | "on the same cable" |
| `CONTROL_PWM_LEFT/RIGHT` + SERIAL on that cable | "on the same cable" |
| `CONTROL_NUNCHUK` + any SERIAL_USART3 | "on the same cable" |

**Not yet enforced** (but still logically wrong): `DEBUG_SERIAL_USARTx` +
`CONTROL_SERIAL_USARTx` on the same port. Both can be defined simultaneously
without a compile error — the host would see ASCII text flowing out while binary
commands come in. Avoid this.

---

## 4. Per-variant UART allocation

The table shows what is **active** on each port for each built-in variant.
"—" means the port is unused (no serial of any kind on that cable).

| Variant | USART2 (LEFT) | USART3 (RIGHT) |
|---------|---------------|----------------|
| **VARIANT_ADC** (no DUAL_INPUTS) | — | `DEBUG_SERIAL_USART3` |
| **VARIANT_ADC** (DUAL_INPUTS) | — | `CONTROL_SERIAL_USART3` + `FEEDBACK_SERIAL_USART3` |
| **VARIANT_USART** | `CONTROL_SERIAL_USART2` + `FEEDBACK_SERIAL_USART2` | — |
| **VARIANT_NUNCHUK** (no DUAL_INPUTS) | `DEBUG_SERIAL_USART2` | — |
| **VARIANT_NUNCHUK** (DUAL_INPUTS) | `CONTROL_SERIAL_USART2` + `FEEDBACK_SERIAL_USART2` | — |
| **VARIANT_PPM** (CONTROL_PPM_RIGHT, no DUAL) | `DEBUG_SERIAL_USART2` | PPM input (not SERIAL) |
| **VARIANT_PWM** (CONTROL_PWM_RIGHT, no DUAL) | `DEBUG_SERIAL_USART2` | PWM input (not SERIAL) |
| **VARIANT_IBUS** (no DUAL_INPUTS) | `DEBUG_SERIAL_USART2` | `CONTROL_SERIAL_USART3` + `FEEDBACK_SERIAL_USART3` |
| **VARIANT_HOVERCAR** | — | `SIDEBOARD_SERIAL_USART3` + `FEEDBACK_SERIAL_USART3` |
| **VARIANT_HOVERBOARD** | `SIDEBOARD_SERIAL_USART2` + `FEEDBACK_SERIAL_USART2` | `SIDEBOARD_SERIAL_USART3` + `FEEDBACK_SERIAL_USART3` |
| **VARIANT_SKATEBOARD** (CONTROL_PWM_RIGHT) | `DEBUG_SERIAL_USART2` | PWM input |

> **Key observation**: most RC-remote variants (`PWM`, `PPM`, `IBUS`, `SKATEBOARD`) put
> `DEBUG_SERIAL_USART2` on the left cable because the right cable is taken by the RC
> receiver. This is the source of the "I see debug text on the left UART" experience
> — you were almost certainly running one of these variants before switching.

---

## 5. Binary frame formats

### 5.1 `SerialCommand` (host → board)

Used when `CONTROL_SERIAL_USARTx` is active. 8 bytes, little-endian.

```c
typedef struct {
  uint16_t start;     // 0xABCD
  int16_t  steer;     // steering command, typically [-1000, 1000]
  int16_t  speed;     // speed    command, typically [-1000, 1000]
  uint16_t checksum;  // start ^ steer ^ speed  (bitwise XOR, all fields as uint16)
} SerialCommand;
```

The board discards frames where `checksum` does not match. The timeout for
missing commands is ~0.8 s (`SERIAL_TIMEOUT = 160` × 5 ms loop).

### 5.2 `SerialFeedback` / `UartReportingFrame` (board → host)

Used when `FEEDBACK_SERIAL_USARTx` is active. Sent via DMA every 10 ms (every
2nd main-loop iteration at 200 Hz). 18 bytes, little-endian.

```c
typedef struct {
  uint16_t start;        // 0xABCD
  int16_t  cmd1;         // echoed steer command (input1[inIdx].cmd)
  int16_t  cmd2;         // echoed speed command (input2[inIdx].cmd)
  int16_t  speedR_meas;  // right motor speed [rpm-like units, vehicle-forward sign: positive == forward]
  int16_t  speedL_meas;  // left  motor speed [rpm-like units, vehicle-forward sign: positive == forward]
  int16_t  batVoltage;   // battery voltage × 100  (e.g. 4200 = 42.00 V)
  int16_t  boardTemp;    // board temperature [°C]
  uint16_t cmdLed;       // LED command flags for sideboard LEDs
  uint16_t checksum;     // XOR of all preceding fields (treated as uint16)
} SerialFeedback;        // also known as UartReportingFrame in core/io/uart_reporting.h
```

Both `speedR_meas` and `speedL_meas` are reported in **vehicle-forward sign convention**: a positive value means the wheel is spinning in the forward direction, matching a positive `cmd2` (speed command). The firmware applies `INVERT_R_DIRECTION` / `INVERT_L_DIRECTION` and the `SPEED_COEFFICIENT` sign before packing, identical to the normalization in `calcAvgSpeed()` and the velocity PI controller.

The checksum covers **all** preceding `uint16`-sized fields:
`start ^ cmd1 ^ cmd2 ^ speedR_meas ^ speedL_meas ^ batVoltage ^ boardTemp ^ cmdLed`.

### 5.3 `SerialSideboard` (sideboard → board)

Used when `SIDEBOARD_SERIAL_USARTx` is active (VARIANT_HOVERBOARD / VARIANT_HOVERCAR).

```c
typedef struct {
  uint16_t start;     // 0xABCD
  int16_t  pitch;     // board angle [°]
  int16_t  dPitch;    // angle derivative
  int16_t  cmd1;      // RC channel 1
  int16_t  cmd2;      // RC channel 2
  uint16_t sensors;   // switches + optical sensors
  uint16_t checksum;
} SerialSideboard;
```

---

## 6. Debug ASCII streams

When `DEBUG_SERIAL_USARTx` is defined, the firmware emits two types of output:

### 6.1 High-frequency telemetry line (every `DEBUG_INPUT_PRINT_INTERVAL_LOOPS`)

```
in1:<raw1> in2:<raw2> rawLong:<x> longOff:<x> rawMinusOff:<x> uCmd:<x>
cmdEff:<x> arm:<x> blk:<x> nz:<x> calibA:<x> calibL:<x> calibI:<x>
vIntent:<x> iMode:<x> zLatchMs:<x> zRel:<x> vSp:<x> aSp:<x> vErr:<x>
vSat:<x> slip:<x> cmdL:<x> cmdR:<x> ErrL:<x> ErrR:<x> BatADC:<x>
BatV:<x> TempADC:<x> Temp:<x> StallL_t:<x> StallR_t:<x> StallSup:<x>
CtrlMode:<x>
```

### 6.2 SetpointTrace line (every `DEBUG_SETPOINT_PRINT_INTERVAL_LOOPS`)

```
SetpointTrace mode:<FWD|REV|ZL> rawLong:<x> longOff:<x> rawLongMinusOff:<x>
uCmd:<x> cmdEff:<x> arm:<x> blk:<x> nz:<x> calibA:<x> calibL:<x> calibI:<x>
vIntent:<x> vSet:<x> aSet:<x> vAct:<x> vErr:<x> vSat:<x> slip:<x>
zLatchMs:<x> zRel:<x> nL:<x> nR:<x> errL:<x> errR:<x> tqL:<x> tqR:<x>
```

Per-wheel fields added for UART speed-direction diagnostics:

| Field | Description |
|-------|-------------|
| `nL`  | Left  wheel measured speed [rpm, vehicle-forward sign] |
| `nR`  | Right wheel measured speed [rpm, vehicle-forward sign] |
| `errL` | Left  speed error = setpoint − nL [rpm] |
| `errR` | Right speed error = setpoint − nR [rpm] |
| `tqL` | Left  torque command output from PI [-1000..1000] |
| `tqR` | Right torque command output from PI [-1000..1000] |

### 6.3 Event-driven lines

Emitted immediately on state changes (not on a fixed interval):

- `CmdFilter calib lock/active/update/torque inhibit ...`
- `Intent mode transition: <old> -> <new> ...`
- `Intent zero-latch armed/active/timer/release ...`
- `MotorErr L:<code>[b0 b1 b2] R:<code>[b0 b1 b2]`
- `SafetyFault ADC/Serial/General timeout ASSERT/CLEAR`
- `SafetyState motors ENABLED/DISABLED`
- `StallDecay state/act ...`

All of these are compiled out entirely when `DEBUG_SERIAL_USARTx` is undefined.

---

## 7. Switching from debug output to binary control+feedback

This is the most common source of confusion when moving from initial bring-up
(where you read ASCII debug text) to autonomous control (where the host sends
commands and reads telemetry).

### The core conflict

`DEBUG_SERIAL_USART2` and `FEEDBACK_SERIAL_USART2` both drive **USART2 TX**,
so they cannot coexist — the compiler enforces this with `#error`.

### How to switch (USART2 / left cable)

**Step 1** — Build with `VARIANT_USART` (PlatformIO):

```ini
; platformio.ini  →  set default_envs or use pio run -e
default_envs = VARIANT_USART
```

or build directly:

```bash
pio run -e VARIANT_USART --target upload
```

In `VARIANT_USART`, `Inc/config.h` defines:
```c
#define CONTROL_SERIAL_USART2  0   // binary command RX on left cable
#define FEEDBACK_SERIAL_USART2     // binary telemetry TX on left cable
// DEBUG_SERIAL_USART2 is intentionally absent
```

`DEBUG_SERIAL_USART3` is also **not** defined in `VARIANT_USART`, so there is
no debug text output on either cable after flashing. The left cable carries
only binary frames.

**Step 2** — What you will now see when you open a terminal:

| Before (debug mode) | After (VARIANT_USART) |
|---------------------|----------------------|
| ASCII text: `SetpointTrace mode:FWD rawLong:0 ...` | Binary garbage: `ÍÍ …` (feedback frames at 10 ms) |
| Human-readable in `screen` | Not human-readable in `screen` — use the test script |

**Step 3** — Use the test script to send commands and read feedback:

```bash
pip install pyserial
python3 tools/scripts/uart_control_test.py --port /dev/ttyUSB0
```

> If you want to keep debug output available while also doing binary control,
> you must use **two separate cables**: binary on the left (USART2) and debug
> text on the right (USART3). Enable `DEBUG_SERIAL_USART3` in addition to
> `CONTROL_SERIAL_USART2` + `FEEDBACK_SERIAL_USART2`. This is not supported by
> any built-in variant directly — you would need a custom config.

---

## 8. `make`-based build default

When building with `make` (instead of PlatformIO), the variant is selected in
`config/feature/feature_flags.h`:

```c
#if !defined(PLATFORMIO)
  // Uncomment exactly ONE variant:
  //#define VARIANT_USART   // binary control+feedback on USART2 (left cable)
  #define VARIANT_PWM     // RC PWM on right cable + DEBUG on left cable  ← default
  ...
#endif
```

The file ships with `VARIANT_PWM` uncommented. `VARIANT_PWM` (with
`CONTROL_PWM_RIGHT`) automatically activates `DEBUG_SERIAL_USART2` — this is
why you see ASCII debug text on the left cable if you built with `make` without
changing that file.

**To use binary control+feedback with `make`**, edit `feature_flags.h` to
comment out `VARIANT_PWM` and uncomment `VARIANT_USART`.

---

## 9. Quick reference: what to expect on each cable

| Scenario | Left cable (USART2) | Right cable (USART3) |
|----------|---------------------|----------------------|
| `VARIANT_PWM` (default make build) | ASCII debug text from `printf` | PWM RC receiver (not UART) |
| `VARIANT_USART` | Binary `SerialFeedback` frames + receives `SerialCommand` | unused |
| `VARIANT_ADC` (no dual) | unused | ASCII debug text |
| `VARIANT_IBUS` | ASCII debug text | Binary control+feedback (iBUS format) |
| `VARIANT_HOVERBOARD` | Binary sideboard + feedback | Binary sideboard + feedback |

---

## 10. Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| ASCII text (`SetpointTrace`, `in1:`, `cmdL:` …) visible on left UART | Built with a variant that has `DEBUG_SERIAL_USART2` (e.g. `VARIANT_PWM`, `VARIANT_PWM` + `CONTROL_PWM_RIGHT`) | Switch to `VARIANT_USART` and reflash |
| `screen` shows nothing on left UART after switching to `VARIANT_USART` | `FEEDBACK_SERIAL_USART2` is active and transmitting binary — correct, but `screen` only shows printable ASCII so frames appear empty or as garbage | Use `uart_control_test.py` to send commands and see decoded feedback |
| Binary garbage on left UART | `FEEDBACK_SERIAL_USART2` is active — correct behaviour for `VARIANT_USART` | Use `uart_control_test.py` to decode frames |
| Build error: "DEBUG_SERIAL_USART2 and FEEDBACK_SERIAL_USART2 not allowed" | Both are defined simultaneously | Remove one — they share the TX wire |
| Board does not respond to commands | Check checksum, verify 115200 8N1, confirm `CONTROL_SERIAL_USART2` is compiled in, check serial timeout | — |
| `uart_control_test.py` shows no feedback | `FEEDBACK_SERIAL_USART2` not active, or TX/RX wires swapped, or DMA stall | Verify wiring: board TX2 → adapter RX, board RX2 ← adapter TX |
| `speedR_meas` or `speedL_meas` in feedback is **opposite sign** to speed command | Sign-convention mismatch between motor physical winding and vehicle-forward convention. Prior to this firmware fix, the raw `n_mot` value was sent without normalisation; the right motor's positive direction is physically "backward". | Confirm you are running the fixed firmware. If a sign inversion still appears after flashing, one motor's Hall sensors or phase wires may be physically reversed — set `INVERT_R_DIRECTION` / `INVERT_L_DIRECTION` accordingly in `Inc/config.h`. |
| Wheels spin opposite to commanded direction | `SPEED_COEFFICIENT` sign bit set inadvertently (bit 16), or `INVERT_x_DIRECTION` wrong for physical motor wiring | Check `SPEED_COEFFICIENT` value (must be `0x4000` = 1.0 for default sense); toggle `INVERT_R_DIRECTION` / `INVERT_L_DIRECTION` and verify `speedR_meas`/`speedL_meas` match command sign |
| Low-speed "clank" or sudden torque bursts at near-zero setpoint | Hall-sensor quantization (±1 rpm) winding the velocity-loop integrator at standstill | Firmware now gates the integrator near zero via `MOTOR_CTRL_STANDSTILL_GATE_RPM` (default 10 rpm). Increase this value in `config/control_tuning/motor_controller_gains.h` if clankiness persists. |
| Wheel feels "frozen" in one direction, then suddenly moves the other way | Integrator windup while setpoint and speed have opposite signs | Fixed by the standstill gate and the existing anti-windup (integrator frozen when saturated in same direction as error). Enable `DEBUG_SERIAL_USART3` alongside binary control on USART2 to watch `errL`/`errR`/`tqL`/`tqR` in the `SetpointTrace` output. |
| Slow ~0.3 Hz oscillation at higher speed (e.g. charger-powered) | Outer-loop PI gains too high for the supply impedance / back-EMF at speed | Reduce `MOTOR_CTRL_VEL_KI_Q15` in `config/control_tuning/motor_controller_gains.h`; also verify battery voltage is stable (charger can cause voltage ripple). |
| `SetpointTrace` shows `nL`/`nR` = 0 even while wheels are spinning | Troubleshooting velocity mode not active, or `MOTOR_LEFT_ENA`/`MOTOR_RIGHT_ENA` not defined | Confirm the troubleshooting block is compiled in (`#ifdef VARIANT_USART` path) and both motor enables are set |
