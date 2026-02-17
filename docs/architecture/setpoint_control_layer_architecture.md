# Setpoint Control Layer Architecture and Implementation Plan

Version 1.1 (repo-aligned, test-gated)

This document adopts the refined **Setpoint Control Layer** architecture and aligns it with the firmware structure currently present in this repository.

## 1. System overview

Target control layering (execution-rate intent):

1. User Command Filtering (1 kHz policy layer)
2. User Intent State Machine (1 kHz policy layer)
3. Velocity Setpoint Control Layer (1 kHz trajectory layer)
4. Motor Controller (1 kHz speed/torque supervision)
5. FOC / Current Controller (ISR domain, ~16 kHz)

Core philosophy:

- Human intent is sharp.
- Physical motion is not.
- The setpoint controller translates sharp intent into physically feasible trajectories.

## 2. Current implementation baseline (what exists today)

Before introducing the new architecture, the current loop already includes command shaping and supervisory behavior that must be preserved or intentionally replaced:

- Fast ISR control and PWM update run in `Src/bldc.c` (`DMA1_Channel1_IRQHandler`) through the generated model integration seam in `core/control/foc_adapter.*`.
- Main-loop command handling currently lives mostly in `Src/main.c` with helper seams in `core/supervisor/*`, `core/io/*`, and `Src/drive_control.c`.
- The present command path already applies filtering/shaping behavior (including low-pass and gain-based shaping terms in the control path) before targets are consumed by the fast loop.

Implementation implication:

- Iteration work must always tie back to this baseline path.
- Every phase needs a test gate proving either parity (no behavior change) or expected intentional behavior change.

## 3. User Command Filtering (1 kHz)

### Responsibility

- Decode PWM/UART command channels.
- Validate plausibility/timeouts.
- Apply minimal smoothing for glitch rejection only.
- Normalize to a common command domain:

`u_cmd ∈ [-1000, 1000]`

### Design notes

- This layer stays hardware-interface centric and does not perform comfort shaping.
- Existing decode/timeout pathways remain the source of truth and are wrapped into a normalized output struct.
- Keep a measurable response target (e.g., step 0→90% in ~100 ms) for regression tracking.

## 4. User Intent State Machine (1 kHz)

### Responsibility

Convert normalized command into sharp desired velocity intent `v_intent`.

### States

- `DRIVE_FORWARD`
- `DRIVE_REVERSE`
- `ZERO_LATCH`

### Behavioral constraints

- Intent transitions remain sharp by design.
- Hard reverse while moving forward must not produce a synthetic brake flag.
- Braking emerges from large `Δv` in downstream setpoint/motor-control layers.
- `ZERO_LATCH` inhibits reverse at near-zero crossing until:
  - speed remains in deadband for 0.5 s, or
  - command sign reverts.

## 5. Velocity Setpoint Control Layer (1 kHz)

### Responsibility

Generate physically feasible `v_sp` from sharp `v_intent` using constrained trajectory evolution.

### Parameters

- `a_max_accel`, `a_max_decel`
- `j_max_accel`, `j_max_decel`

Use asymmetric limits (decel stronger than accel).

### Trajectory logic

Given `v_t`, `a_t`, `v_intent`:

1. Evolve acceleration under jerk constraints.
2. Predict overshoot if current acceleration is maintained.
3. If overshoot risk exists, ramp acceleration toward zero.

This yields an S-curve trajectory without injecting additional intent lag.

### Constraint: shaping should not depend on noisy feedback

Nominal shaping state uses `v_sp`, `a_sp`, and `v_intent`.

### Windup/release protection (required exception)

To avoid “stuck in mud → surge on release,” enforce:

`||v_sp| - |v_actual|| < k_slip_limit`

When exceeded:

- freeze or reduce setpoint growth,
- optionally reduce effective accel bound,
- expose active flag to telemetry.

This is a physical plausibility/safety guard, not intent reinterpretation.

## 6. Motor Controller (1 kHz)

### Responsibility

- Speed loop (`v_sp` to torque/current request).
- Stall detection and recovery supervision.
- Soft protection and torque limiting.
- Current envelope management.

### Hard vs soft limit policy (explicit)

- **Hard current limit/fault behavior lives in generated control and is not changed.**
- Architecture/iteration work only modifies **soft-limit behavior above generated FOC**, such as speed-dependent limiting or ramp constraints before requests are handed to the generated layer.

Repository note: current responsibilities are split across main-loop supervision and controller-facing plumbing; iteration work should improve boundary clarity without touching generated hard-limit internals.

## 7. FOC / Current Controller (~16 kHz ISR)

FOC remains a dedicated low-level electrical control layer:

- current control,
- transforms/modulation,
- PWM generation,
- electrical protection.

It stays isolated from intent/state-machine policy.

Repository note: preserve the `foc_adapter` seam and avoid direct generated-code feature edits unless strictly necessary.

## 8. Hall speed estimation (KISS)

Keep estimator simple/deterministic:

- 3 halls, 15 pole pairs, 90 transitions/mech rev, ~4° mech resolution.
- `omega = K / dt_edge` between edges.
- `omega = 0` after 125 ms timeout without edge.

No observer stack is required for this phase.

## 9. Critical repository-level review

1. **Layer naming and ownership clarity**
   - “Setpoint Control Layer” is the right name and cleanly separates intent vs motion shaping.

2. **Hard reverse semantics**
   - No explicit braking mode is consistent with physics-first behavior and reduces mode complexity.

3. **FOC separation**
   - Strong match with existing ISR/main split; preserve this boundary.

4. **Windup/surge risk**
   - Setpoint-only shaping can store latent delta-v during low traction; slip-gap constraint is mandatory in first usable release.

5. **Current-loop baseline continuity**
   - Existing low-pass/gain shaping in the current loop path is part of today’s behavior baseline and must be covered by parity checks before replacement.

## 10. Implementation plan (test-gated, practical)

To address integration confidence, use fewer phases with explicit "test moments" between each phase.

### Phase A — baseline extraction with behavior parity

Scope:

- Introduce `command_filter`, `intent_state_machine`, and `velocity_setpoint_layer` module interfaces.
- Wire them in pass-through mode so effective behavior still matches current path.
- Add telemetry channels for current path outputs and proposed new-layer outputs side-by-side.

Test gate A (must pass before Phase B):

- Build/flash/runtime sanity.
- Command-step replay shows parity vs pre-refactor baseline within agreed tolerance.
- Timeout/failsafe behavior unchanged.

### Phase B — intent-state activation (no trajectory shaping yet)

Scope:

- Activate `DRIVE_FORWARD/DRIVE_REVERSE/ZERO_LATCH` intent logic.
- Keep legacy smoothing path for physical command shaping.
- Log intent transitions and zero-latch timer decisions.

Test gate B:

- Hard reverse command causes immediate intent sign change.
- No synthetic brake flag introduced.
- Zero-crossing latch behavior deterministic and repeatable.

### Phase C — setpoint trajectory activation (jerk/accel asymmetry)

Scope:

- Enable jerk-limited/asymmetric `v_sp` generator.
- Keep hard limits untouched in generated code.
- Keep soft-limit path external to generated layer.

Test gate C:

- `v_sp` traces show expected S-curve.
- Emergency reverse yields strong controlled decel without mode artifacts.
- No instability near zero crossing.

### Phase D — slip-gap anti-windup + soft-limit tuning integration

Scope:

- Enable `||v_sp|-|v_actual||` clamp and release policy.
- Integrate/retune soft-limit strategy in this same phase (still outside generated hard-limit logic).
- Finalize parameter placement in `config/control` and `config/user`.

Test gate D:

- Stuck-then-release scenario does not cause surge.
- Soft-limit intervention is visible/traceable and does not weaken hard-limit safety behavior.
- End-to-end road test matrix passes acceptance criteria.

## 11. Tuning strategy (operator-focused)

### 11.1 Tuning order

1. Zero-latch/deadband timing.
2. `a_max_decel` then `a_max_accel`.
3. `j_max_decel` then `j_max_accel`.
4. `k_slip_limit` and clamp-release policy.
5. Soft-limit envelope parameters.

### 11.2 Practical test flow between phases

- Bench replay first (deterministic input sequence).
- Controlled low-speed track test.
- Emergency reverse test at increasing speed levels.
- Low-traction/stall-release test for anti-surge verification.
- Final mixed maneuver regression pass.

### 11.3 Required telemetry

- `u_cmd`, validity, filter outputs.
- Intent state and `v_intent`.
- `v_sp`, `a_sp`, trajectory phase flags.
- `v_actual`, slip-gap metric, clamp active flag.
- Speed-loop error and torque/current request.
- Soft-limit active flag and effective soft-limit value.

### 11.4 Acceptance criteria

- No hidden mode jumps at sign change.
- No oscillatory behavior near zero.
- No post-release surge after prolonged stall/low traction.
- Emergency reverse consistently maps to strongest available controlled decel.
- Hard-limit/fault behavior remains generated-code owned and unchanged.

## 12. Summary

This repository is already structurally compatible with the Setpoint Control Layer architecture due to the existing ISR/main split and `foc_adapter` seam. The most reliable path is a **test-gated phase rollout** with explicit parity checkpoints, clear separation of hard vs soft limits, and first-class anti-windup handling.
