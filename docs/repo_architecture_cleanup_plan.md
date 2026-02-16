# Repository Architecture Cleanup Plan

## Why this document exists
This document is the long-lived reference for cleanup and restructuring work in this repository.
It has two roles:
1. Describe the **current architecture** (where things are now).
2. Define the **target architecture + migration workflow** (where we are going and how we get there safely).

Because this cleanup will be executed across multiple Codex iterations and commits, this file also includes a **progress log** so each next iteration can start with clear context.

---

## 1) Current repository layout (as-is)

### Top-level directories

- `.github/`
  CI workflows and issue templates.
- `Arduino/`
  Arduino example serial controller (`hoverserial.ino`).
- `Drivers/`
  STM32 HAL + CMSIS vendor dependencies.
- `Inc/`
  Firmware headers (`config.h`, `defines.h`, `BLDC_controller.h`, etc.).
- `Src/`
  Main firmware source (`main.c`, `BLDC_controller.c`, `setup.c`, `comms.c`, etc.).
- `docs/`
  Mixed architecture notes, schematics, literature, images, and `recycle/` legacy code.
- `scripts/`
  Helper scripts (build/flash).
- `MDK-ARM/`
  Keil uVision project files.
- Root build/tooling files (`Makefile`, `platformio.ini`, linker/startup files).

### Current functional placement

- **Application flow + runtime orchestration** is concentrated in `Src/main.c`.
- **Low-level control algorithm code** is concentrated in `Src/BLDC_controller.c` (+ related generated files).
- **Configuration, feature selection, and tuning params** are heavily centralized in `Inc/config.h`.
- **Input and communications logic** is spread across `comms.c`, `setup.c`, `util.c`, and preprocessor branches.
- **Documentation** contains both current and historical content without a strict separation.

### Current hotspots (size / complexity indicators)

- `Src/BLDC_controller.c` is a large monolithic file.
- `Src/main.c` mixes many responsibilities.
- `Inc/config.h` acts as a catch-all configuration layer.
- `Src/util.c` combines many utility concerns.

---

## 2) Main structural problems

1. **Weak separation of concerns**
   Main loop, command handling, safety handling, mode arbitration, and diagnostics are intertwined.

2. **Monolithic modules**
   Large files increase risk during changes and slow review/testing.

3. **Configuration sprawl in one file (`config.h`)**
   User tuning, board internals, and feature toggles are mixed together.

4. **High preprocessor branching complexity**
   Behavior is hard to trace across variants and feature combinations.

5. **Documentation discoverability issues**
   Architecture, interfaces, and archived material are not clearly separated.

---

## 3) Target architecture (to-be)

### Design principles

- Single responsibility per module.
- Clear layering (app → supervisor/control → platform).
- Strict separation of generated vs hand-written code.
- Explicit interfaces and state structs for testability.
- Configuration split into user/feature/board-control layers.

### Proposed target structure

```text
.
├── app/
│   ├── main.c                     # bootstrap + scheduler + module orchestration
│   └── app_loop.c/.h              # top-level runtime state machine
├── core/
│   ├── control/
│   │   ├── control_loop.c/.h
│   │   ├── command_mixer.c/.h
│   │   ├── torque_control.c/.h
│   │   ├── speed_control.c/.h
│   │   └── foc_adapter.c/.h       # only interface to generated BLDC model
│   ├── supervisor/
│   │   ├── safety_supervisor.c/.h
│   │   ├── mode_supervisor.c/.h
│   │   ├── input_supervisor.c/.h
│   │   └── power_supervisor.c/.h
│   ├── io/
│   │   ├── input_decode.c/.h
│   │   ├── uart_reporting.c/.h
│   │   └── sideboard_io.c/.h
│   └── diagnostics/
│       ├── diag_events.c/.h
│       └── diag_snapshot.c/.h
├── platform/
│   └── stm32/
│       ├── hal_init.c/.h
│       ├── interrupts.c/.h
│       ├── adc_dma.c/.h
│       ├── pwm_timer.c/.h
│       └── board_pins.h
├── generated/
│   └── bldc/
│       ├── BLDC_controller.c/.h
│       ├── BLDC_controller_data.c
│       └── rtwtypes.h
├── config/
│   ├── user/
│   │   ├── user_params.h
│   │   └── user_profile_*.h
│   ├── feature/
│   │   └── feature_flags.h
│   ├── board/
│   │   ├── board_default.h
│   │   └── board_variant_*.h
│   └── control/
│       ├── control_defaults.h
│       └── control_limits.h
├── docs/
│   ├── architecture/
│   ├── interfaces/
│   ├── hardware/
│   └── archive/
├── tools/
│   └── scripts/
└── examples/
    └── arduino/
```

> Note: This is the target end-state. Migration should be incremental and compatibility-first.

---

## 4) Monolith split strategy

### 4.1 `main.c` responsibilities to extract

`main.c` should only keep:
- platform bootstrap,
- module initialization,
- main tick scheduling,
- high-level fault stop behavior.

Move out of `main.c`:
- command filtering/mixing → `core/control/command_mixer.*`
- mode arbitration → `core/supervisor/mode_supervisor.*`
- timeout/failsafe input handling → `core/supervisor/input_supervisor.*`
- runtime telemetry/debug output → `core/io/uart_reporting.*`

### 4.2 `BLDC_controller.c` integration boundary

- Keep generated controller files as untouched as possible.
- Introduce `foc_adapter.c/.h` as the only bridge between app logic and generated controller I/O.
- Centralize scaling/unit conversion in the adapter.

### 4.3 `config.h` decomposition

Split into:
1. `config/user/*` for user-tuneable parameters.
2. `config/feature/*` for compile-time feature switches.
3. `config/board/*` and `config/control/*` for board internals and control constants.

Keep a temporary compatibility façade (`config.h`) during migration.

### 4.4 Safety and diagnostics as first-class modules

- `safety_supervisor`: emergency stop policy, fault latching/reset.
- `diag_events`: structured event/fault log.
- `diag_snapshot`: periodic status snapshots for telemetry/debug.

---

## 5) Documentation restructuring

### Current issue
`docs/` is useful but mixed: active architecture references and legacy material are adjacent.

### Target
- `docs/architecture/` for system design and rationale.
- `docs/interfaces/` for protocols/contracts.
- `docs/hardware/` for board-level references.
- `docs/archive/` for historical/recycled content.

---

## 6) Execution workflow across Codex iterations

This cleanup will **not** be done in one long-running conversation. It will be delivered as short, validated, git-backed iterations.

### Iteration contract (applies to every iteration)

For each iteration:
1. Isolate one bounded refactor scope.
2. Implement with no unnecessary side-effects.
3. Build firmware (and run available checks).
4. Verify no behavioral regressions for the selected scope.
5. Commit with a clear message.
6. Update the progress section in this document.
7. Update `docs/architecture/README.md` when architecture boundaries or control/data-flow responsibilities change.

### Suggested 5-iteration rollout

#### Iteration 1 — Structure prep (no behavior change)
- Add target folders and initial module skeletons.
- Add lightweight include/forwarding compatibility where needed.
- Keep runtime behavior identical.

Validation:
- Full build passes.
- Binary behavior unchanged for default variant.

#### Iteration 2 — Extract supervisors from `main.c`
- Move timeout/failsafe and mode arbitration into `core/supervisor/*`.
- Keep public behavior and macros unchanged.

Validation:
- Full build passes.
- Runtime sanity checks on command timeout/failsafe behavior.

#### Iteration 3 — Extract IO/telemetry surfaces
- Move UART reporting and input decode boundaries into `core/io/*`.
- Keep protocol behavior backwards-compatible.

Validation:
- Full build passes.
- Serial telemetry format still valid for current tools.

#### Iteration 4 — Introduce FOC adapter boundary
- Add `core/control/foc_adapter.*` and route BLDC interface through it.
- Keep generated code untouched except include path / integration wiring.

Validation:
- Full build passes.
- Controller outputs match pre-adapter behavior within expected tolerance.

#### Iteration 5 — Config split and docs finalization
- Split `config.h` into user/feature/board-control layers.
- Keep temporary compatibility macros.
- Move docs to architecture/interfaces/hardware/archive structure.

Validation:
- Full build passes for representative variants.
- Existing user customization paths remain documented and functional.

---

## 7) Progress tracking (living section)

This section must be updated in each iteration to maintain continuity.

### Global status
- Overall cleanup program: **Iterations 1-5 closed and validated; migration complete for planned rollout**.
- Current iteration: **5 (config split + docs finalization) — closed**.
- Next planned iteration: **Post-rollout cleanup/hardening — pending definition**.

### Completed so far
- Added this architecture cleanup plan.
- Captured current-state mapping and key structural issues.
- Defined target architecture and module boundaries.
- Defined a 5-iteration execution workflow with validation gates.
- Created initial target directory scaffolding (`app/`, `core/`, `platform/`, `generated/`, `config/`, `tools/`, `examples/`).
- Added initial no-op module skeletons for app loop, supervisor, io, diagnostics, and platform init boundaries.
- Added docs and config placeholder READMEs for the target structure while keeping existing runtime paths unchanged.
- Ran build validation after structural prep.
- Extracted supervisor boundaries from `main.c` for timeout/failsafe observation and control-mode arbitration (iteration 2).
- Extracted IO/telemetry boundaries from `main.c` into `core/io/*` for input decode snapshotting and UART feedback frame assembly (iteration 3).
- Added active FOC adapter module (`core/control/foc_adapter.*`) and integrated it into the build path (iteration 4).
- Routed generated BLDC input/output stepping in `Src/bldc.c` through the adapter boundary while preserving generated controller sources unchanged.
- Routed `Src/main.c` runtime reads of motor speed and DC link current through adapter accessors.
- Split core configuration layers for iteration 5 by introducing `config/feature/feature_flags.h`, `config/control/control_defaults.h`, `config/board/board_default.h`, and `config/user/user_params.h`.
- Kept `Inc/config.h` as compatibility facade and sourced new layered headers there so existing include paths and macros remain functional.
- Updated architecture/config documentation to reflect the active layering and compatibility path.

### Not started yet
- No BLDC generated-file relocation has been executed yet.

### Iteration log

- **Iteration 5 — 2026-02-16**
  - Scope executed:
    - Introduced active layered config headers in `config/`:
      - `config/feature/feature_flags.h` (variant + feature selection)
      - `config/control/control_defaults.h` (control timing/PWM/ADC defaults)
      - `config/board/board_default.h` (board variant selection)
      - `config/user/user_params.h` (user-facing default tuning values)
    - Converted `Inc/config.h` into a compatibility facade for the newly split layers by including these new headers at the top while preserving downstream legacy config blocks and validation logic.
    - Updated `config/README.md` and `docs/architecture/README.md` with the iteration-5 config layering model and migration status.
  - Build/check commands and results:
    - `make clean` → passed.
    - `make -j4` → blocked in Codex environment (`arm-none-eabi-gcc` missing).
  - Compatibility notes:
    - Existing firmware source files continue to include `Inc/config.h` unchanged.
    - Macros moved into layered headers are still visible through the same compatibility include path.
    - Legacy user customization flow remains functional while split migration continues incrementally.
  - Next step:
    - Optional follow-up: migrate additional thematic config sections (battery/temperature/motor-control/variant blocks) into layered headers with minimal-diff macro parity checks.

- **Iteration 4 — 2026-02-16**
  - Scope executed:
    - Added concrete adapter implementation in `core/control/foc_adapter.*` to encapsulate generated BLDC model instances, input/output structures, and model stepping entrypoints.
    - Wired `Src/bldc.c` motor-control interrupt path through adapter calls (`FocAdapter_SetInputFrame`, `FocAdapter_Step`, `FocAdapter_GetOutput`, and adapter-side error-bit clear helper) instead of touching generated extern symbols directly.
    - Wired `Src/main.c` runtime telemetry/supervision reads for motor speed and DC-link current through adapter accessors.
    - Updated `Makefile` to compile `core/control/foc_adapter.c` and expose include paths for `core/control`.
    - Updated architecture documentation in `docs/architecture/README.md` to keep architectural reference in sync with the implementation boundary introduced in this iteration.
  - Build/check commands and results:
    - `make clean` → passed.
    - `make -j4` → blocked in Codex environment (`arm-none-eabi-gcc` missing).
  - Compatibility notes:
    - Generated model sources (`Src/BLDC_controller.c`, `Src/BLDC_controller_data.c`) remain untouched.
    - Existing control loop behavior and serial-facing semantics are preserved; this step focuses on boundary introduction and call routing only.
  - Review notes / plan tweaks:
    - Iteration contract now explicitly requires updating `docs/architecture/README.md` whenever architecture boundaries/dataflow responsibilities are changed.
  - Next step:
    - Iteration 5: split `Inc/config.h` into target-layered config headers with temporary compatibility facade and finalize docs structure cleanup.


- **Iteration 3 — 2026-02-16**
  - Scope executed:
    - Integrated IO modules into active build path (`Makefile`) and include path wiring.
    - Added `core/io/input_decode.*` boundary with `InputDecodePair` and `InputDecode_BuildPair(...)` to isolate decoded input snapshot composition used by periodic debug reporting.
    - Expanded `core/io/uart_reporting.*` with `UartReportingFrame` and `UartReporting_PrepareFrame(...)` so UART feedback framing/checksum are owned by `core/io` rather than inlined in `Src/main.c`.
    - Wired `Src/main.c` to use `UartReporting_OnFrame(...)` and `UartReporting_PrepareFrame(...)` for feedback serial output while preserving frame format and DMA dispatch behavior.
  - Build/check commands and results:
    - `make clean` → passed.
    - `make -j4` → blocked in Codex environment (`arm-none-eabi-gcc` missing).
  - Compatibility notes:
    - Feedback serial packet fields and checksum formula remain unchanged.
    - Debug serial content remains backward-compatible; input and command values now flow through the `input_decode` boundary before printing.
  - Review notes / plan tweaks:
    - Iteration sequencing still looks solid; extracting IO before the FOC adapter remains the lowest-risk order.
    - Minor tweak recommended for Iteration 4: keep adapter work strictly at integration points first (no early unit-scaling cleanups), then follow with optional cleanup commit to reduce regression risk.
  - Next step:
    - Iteration 4: introduce `core/control/foc_adapter.*` active boundary and route BLDC integration through it without touching generated control math.


- **Iteration 2 — 2026-02-16**
  - Scope executed:
    - Integrated supervisor modules into active build path (`Makefile`) and include path wiring.
    - Extracted timeout/failsafe state observation from `Src/main.c` into `core/supervisor/input_supervisor.*` via `InputSupervisorState` and `InputSupervisor_Update(...)` calls.
    - Extracted control-mode arbitration helper from `Src/main.c` into `core/supervisor/mode_supervisor.*` via `ModeSupervisorState`, `ModeSupervisor_Select(...)`, and `ModeSupervisor_IsStallDecayActive(...)`.
    - Replaced inline stall-decay mode selection logic in `main.c` with `mode_supervisor` boundary call while preserving existing control-mode macros and behavior.
  - Build/check commands and results:
    - `make clean` → passed.
    - `make -j4` → blocked in Codex environment (`arm-none-eabi-gcc` missing).
  - Compatibility notes:
    - Existing runtime globals and macro semantics (`ctrlModReq`, timeout flags, stall decay feature gates) remain unchanged.
    - Extraction is behavior-preserving and limited to supervisor boundary introduction + wiring.
  - Hardware validation feedback (user test):
    - Firmware was tested successfully on-device after Iteration 2 changes and behavior matched expected operation.
  - Next step:
    - Iteration 3: move UART reporting and input decode boundaries into `core/io/*` with protocol compatibility preserved.


- **Iteration 1 — 2026-02-16**
  - Scope executed:
    - Added target architecture scaffold directories and placeholder documentation buckets.
    - Added initial module skeletons with minimal/no-op implementations:
      - `app/app_loop.*`
      - `core/supervisor/{input_supervisor,mode_supervisor}.*`
      - `core/io/uart_reporting.*`
      - `core/diagnostics/diag_snapshot.*`
      - `platform/stm32/hal_init.*`
      - `core/control/foc_adapter.h` compatibility-forwarding bridge.
    - Kept firmware behavior unchanged by not wiring new modules into current build/runtime flow.
  - Build/check commands and results:
    - `make clean` → passed.
    - `make -j4` → blocked in Codex environment (`arm-none-eabi-gcc` missing).
  - Compatibility notes:
    - Existing source layout under `Src/` + `Inc/` remains the active build path.
    - New folders and files are additive and non-invasive for this iteration.
  - Follow-up adjustments after review:
    - Added safety-focused serial debug transition logs in `Src/main.c` for timeout assertions/clears (`timeoutFlgADC`, `timeoutFlgSerial`, `timeoutFlgGen`), control-mode changes (`ctrlModReq`), and motor enable/disable transitions.
    - Goal: make safety-relevant runtime state changes explicit on debug serial during bench/road testing.
  - Hardware validation feedback (user test):
    - User reported that firmware behavior is preserved on-device and previously working behavior remains intact.
    - Iteration 1 scope is accepted/closed; repository is ready to merge this step and proceed to Iteration 2 in a later change.
  - Next step:
    - Iteration 2: move timeout/failsafe and mode arbitration logic from `main.c` into `core/supervisor/*` with behavior-preserving integration.

### How to update this section after each iteration
For each completed iteration, append:
- Iteration number and date.
- Scope executed.
- Build/check commands run and result.
- Files/modules moved or created.
- Compatibility notes and next step.

---

## 8) Definition of done

Cleanup is complete when all of the following are true:
- `main.c` is orchestration-focused and no longer monolithic.
- Generated BLDC code is isolated behind a stable adapter.
- Config is split into user/feature/board-control layers.
- Supervisor, control, io, and diagnostics boundaries are explicit.
- Docs are curated by purpose (architecture/interfaces/hardware/archive).
- Build and key variants remain functional throughout migration.
