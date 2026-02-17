# Firmware architecture (implementation-focused)

This document explains how the current firmware implementation is organized and how control/data move through it at runtime. It is intended as a practical architecture reference for engineering discussions.

Additional architecture design docs:

- `docs/architecture/setpoint_control_layer_architecture.md`: target layering and iterative implementation plan for the setpoint-control-focused command path.
- Legacy completed proposal moved to recycle bin: `docs/recycle/torque_mode_accel_and_steering_architecture.md`.

## 1. System intent and runtime model

The firmware runs a dual-motor controller on STM32F103 hardware. The runtime has two dominant execution domains:

- **Fast interrupt domain (~16 kHz, DMA/ADC driven):** samples motor/board signals, executes motor control (generated BLDC algorithm), and updates phase PWM outputs.
- **Main loop domain (cooperative, millisecond-scale):** decodes command inputs, arbitrates drive behavior/modes/safety policy, updates user-facing telemetry and diagnostics, and prepares command targets used by the fast loop.

The architecture is therefore not a generic layered application in the abstract; it is a deterministic embedded split between **hard real-time control execution** and **supervisory/application logic**.

## 2. Module map (current implementation)

### 2.1 Entry points and orchestration

- `Src/main.c`
  - Initializes HAL/peripherals and firmware subsystems.
  - Runs the supervisory control loop: input processing, command shaping/mixing, timeout/failsafe policy wiring, telemetry/debug updates.
  - Produces global command targets (`pwml`, `pwmr`) consumed by the real-time control ISR.

- `Src/bldc.c`
  - Owns the DMA/ADC ISR (`DMA1_Channel1_IRQHandler`).
  - Reads ADC + hall signals, applies low-level electrical protections, and executes left/right motor control steps.
  - Applies controller outputs to timer PWM compare registers.

### 2.2 Generated control model boundary

- Generated model files remain under `Src/BLDC_controller.c`, `Src/BLDC_controller_data.c`, `Inc/BLDC_controller.h`.
- `core/control/foc_adapter.h/.c` is the active integration boundary between hand-written firmware and generated controller symbols.
  - Encapsulates: model/input/output selection, input-frame population, step dispatch, and output/error accessors.
  - Keeps generated code untouched while allowing the rest of the firmware to depend on a stable hand-written interface.

### 2.3 Supervisor and IO extraction boundaries (from iterations 2-3)

- `core/supervisor/input_supervisor.*`
  - Encapsulates timeout/failsafe state observation helpers previously inlined in `main.c`.

- `core/supervisor/mode_supervisor.*`
  - Encapsulates control-mode/stall-decay arbitration helpers.

- `core/io/input_decode.*`
  - Encapsulates input pair snapshots for serial/debug usage.

- `core/io/uart_reporting.*`
  - Encapsulates UART feedback-frame construction and reporting state transitions.

These modules are now active in the build and used by `main.c`, reducing direct monolithic logic density.

## 3. Control and data flow

## 3.1 Fast control path (ISR)

`DMA1_Channel1_IRQHandler` in `Src/bldc.c` executes the motor-control pipeline:

1. **Sensor and offset handling**
   - ADC offset calibration (startup phase).
   - Battery filtering and current signal acquisition.

2. **Electrical protection gates**
   - Current chopping via PWM MOE disable if DC-link current exceeds configured bounds.

3. **Input-to-controller binding (via adapter)**
   - Build per-motor `FocAdapterInputFrame` from latest command + hall/current signals.
   - Push frames through `FocAdapter_SetInputFrame(...)`.

4. **Controller step dispatch (via adapter)**
   - Call `FocAdapter_Step(...)` per enabled motor.

5. **Output application**
   - Fetch output structures through `FocAdapter_GetOutput(...)`.
   - Write phase duty values to timer channels with clamp/margin logic.

6. **Fault/recovery plumbing**
   - Read error codes via `FocAdapter_GetErrorCode(...)`.
   - Maintain stall-recovery grace window and clear stall bits via adapter helper when recovery conditions are met.

This path is deliberately minimal in abstraction depth beyond the adapter to preserve deterministic timing.

## 3.2 Main-loop supervisory path

The main loop in `Src/main.c` performs slower-rate policy and user-interface logic:

1. Input acquisition/conditioning.
2. Supervisor-based timeout/mode arbitration.
3. Command shaping (speed/steer, drive control, decay helpers).
4. Telemetry/reporting frame preparation through `core/io` modules.
5. Runtime metric calculation (including motor speed/DC-link current reads now accessed through `foc_adapter`).

Outputs of this path are command targets and state flags that the ISR consumes.

## 4. Architectural boundaries and responsibilities

### 4.1 What is intentionally isolated

- **Generated control math and state layout** are not edited directly in normal feature refactors.
- **Integration details** (which generated globals/models are used and how) are localized in `core/control/foc_adapter.*`.

### 4.2 Current config layering (iteration 5)

- `Inc/config.h` remains the stable compatibility facade include for the firmware codebase.
- The facade now composes layered config headers:
  - `config/feature/feature_flags.h` (variant/feature selection).
  - `config/control/control_defaults.h` (PWM/ADC/timing defaults used by control runtime).
  - `config/board/board_default.h` (board mapping defaults).
  - `config/user/user_params.h` (user-tunable default values).
- Legacy config blocks that are still in `Inc/config.h` continue to function unchanged, so existing customization paths remain valid while migration proceeds.

### 4.3 What remains centralized (for now)

- `Src/main.c` still contains significant application logic, though supervisor/io seams are now established.

## 5. Key globals and ownership (practical view)

- Command targets (`pwml`, `pwmr`) are produced in main loop and consumed in ISR.
- Motor enable/fault-effective behavior is finalized inside the ISR path (`enableFin`, stall recovery state).
- Generated model parameters/states remain owned by generated structures; access should prefer adapter indirection unless code is itself part of model initialization internals.

## 6. Why this matters for future changes

For upcoming cleanup work (especially deeper module extraction), use this rule:

- If a change touches generated BLDC I/O or model stepping, it should flow through `core/control/foc_adapter.*` first.
- If a change alters high-level behavior sequencing (timeouts, mode switches, command shaping), prefer supervisor/control modules over adding new logic directly into `main.c`.

This keeps migration incremental while preserving runtime behavior and testability.
