# Driver-Intent Braking Module Architecture (Design Only)

## 1) Objective

Add a **driver-intent layer** on top of the existing command shaping/mixing path that improves braking behavior without destabilizing steering or low-level FOC.

Primary goals:

1. Keep current smooth ramp-up behavior for positive drive intent.
2. Make ramp-down behavior significantly faster than ramp-up.
3. Detect intentional braking when the user quickly releases forward input to neutral.
4. Base brake decisions on **combined vehicle velocity** from both wheels so steering differentials do not falsely trigger braking.
5. Bring speed to zero with controlled regen/negative torque (no wheel lock or abrupt stop).
6. Handle the fact that joystick input used here is **raw/unfiltered and includes offset**, unlike downstream filtered wheel commands.

This document defines architecture, signal placement, state logic, and tuning strategy only (no firmware changes yet).

---

## 2) Placement in the control pipeline

Recommended placement in the existing main-loop supervisory flow:

1. Read raw joystick input (`speed_raw`, `steer_raw`) from input decode.
2. Apply **intent preprocessing** for raw offset/deadband normalization.
3. Run new **DriverIntentBraking** module (state machine + shaping policy).
4. Output an intent-conditioned `speed_intent` and optional `brake_torque_request`.
5. Continue into existing filtering/mixing and wheel-command supervisor stages.

Rationale:

- Placing it too late (after per-wheel mixing) makes steering asymmetry look like braking intent.
- Placing it too early without offset handling makes false transitions likely due to joystick bias/noise.
- Keeping it above wheel-level modules preserves low-level safety/protection boundaries and avoids modifying generated FOC internals.

---

## 3) Proposed module boundaries

Create a new conceptual module:

- `core/supervisor/driver_intent_braking.h/.c` (or equivalent control-layer location)

### Inputs

- `speed_raw` (unfiltered joystick speed command, includes offset)
- `steer_raw` (unfiltered joystick steer command)
- `speed_combined_rpm` (signed combined vehicle speed from two wheels)
- `speed_left_rpm`, `speed_right_rpm` (optional for diagnostics only)
- `dt_s` (explicit loop timestep)
- mode flags (drive enable, timeout/failsafe status, selected ctrl mode)

### Outputs

- `speed_cmd_shaped` (intent-conditioned longitudinal command)
- `brake_active` flag
- `intent_state` enum
- optional: `brake_strength` [0..1]

### Internal state

- calibrated joystick neutral offset estimate
- normalized speed command and derivative
- previous normalized speed command
- timers/counters for release detection and hold windows
- filtered combined speed estimate
- state machine state

---

## 4) Intent preprocessing (raw command handling)

Because this module consumes raw joystick values, add a strict preprocessing stage:

1. **Neutral offset compensation**
   - Maintain a slow estimator for speed channel neutral only when conditions indicate true neutral (vehicle nearly stopped + command stable).
   - Do not adapt offset while moving or while command magnitude is high.

2. **Deadband + normalization**
   - Convert raw command into `u_speed_norm` in `[-1, +1]` after offset subtraction.
   - Apply a small deadband near zero to suppress noise-induced intent toggles.

3. **Command derivative**
   - Compute `du_speed/dt` to detect fast release events (forward to neutral behavior).

This ensures downstream intent logic sees stable semantics independent of joystick bias.

---

## 5) Driver-intent state machine

Use a compact explicit state machine.

### States

1. **NEUTRAL_COAST**
   - No active drive/brake intent.
   - Command around neutral and low combined speed.

2. **DRIVE_APPLY**
   - Positive or negative propulsion intent follows normal ramp-up rules.

3. **FAST_DECEL_REQUEST**
   - User has reduced propulsion demand; apply faster ramp-down than ramp-up.

4. **BRAKE_INTENT_ACTIVE**
   - Triggered by a rapid forward->neutral release while vehicle speed magnitude is above threshold.
   - Apply controlled braking torque profile until near-stop.

5. **BRAKE_HOLD_TAPER**
   - Near zero speed, taper braking to avoid oscillation/chatter; transition to neutral.

### Key transitions

- `NEUTRAL_COAST -> DRIVE_APPLY`
  - `|u_speed_norm| > drive_enter_threshold`

- `DRIVE_APPLY -> FAST_DECEL_REQUEST`
  - Command magnitude decreases (`|u_now| < |u_prev|`) or sign moves toward zero.

- `DRIVE_APPLY -> BRAKE_INTENT_ACTIVE`
  - Forward command was above threshold,
  - rapid release detected (`du_speed/dt < -release_rate_threshold`),
  - current command near neutral,
  - `|speed_combined_rpm| > min_speed_for_brake_intent`.

- `BRAKE_INTENT_ACTIVE -> BRAKE_HOLD_TAPER`
  - combined speed near zero band or max brake-intent window elapsed.

- `BRAKE_HOLD_TAPER -> NEUTRAL_COAST`
  - speed and command remain in neutral band for hold time.

- Any state -> safety fallback
  - timeout/fault/disable: clear braking intent and output zero-safe command.

---

## 6) Asymmetric ramping policy (core requested behavior)

Replace single symmetric command LPF behavior conceptually with asymmetric dynamics:

- **Ramp-up**: keep existing feel (current rate/LPF constants).
- **Ramp-down**: use faster decay path (higher rate limit and/or lower LPF time constant).

Implementation concept (architecture-level):

- If requested command magnitude increases: apply `tau_up` / `rate_up`.
- If requested command magnitude decreases: apply `tau_down` / `rate_down`, where `tau_down << tau_up` and/or `rate_down >> rate_up`.

This can be implemented as either:

1. Two-sided rate limiter (different positive/negative slew for magnitude change), or
2. Direction-aware LPF with two coefficients.

For predictability, prefer **rate limiter first, then LPF** with asymmetric parameters.

---

## 7) Brake-intent generation logic

When release-to-neutral intent is detected, synthesize braking as controlled negative torque demand (not mechanical lock).

### Detection inputs

- normalized command current/previous
- command derivative
- combined speed magnitude
- debounce timer

### Brake command profile

- Start with moderate brake strength (speed-proportional).
- Limit by configurable max regen/brake torque.
- Reduce as speed approaches zero (taper zone).
- Cancel immediately if user reapplies propulsion command beyond threshold.

Example conceptual law (not code):

- `brake_strength = clamp(k_v * |v_combined| + k_release * release_event_score, 0, brake_max)`
- `speed_cmd_shaped = -sign(v_combined) * brake_strength` while in `BRAKE_INTENT_ACTIVE`

This naturally commands deceleration opposite current travel direction.

---

## 8) Combined velocity strategy (anti-false-trigger with steering)

To avoid mistaking steering-induced wheel split for braking:

1. Use a **combined longitudinal velocity estimate** as the authoritative brake-intent speed signal.
2. Compute from both wheel motor speeds with proper sign conventions already used in firmware.
3. Optionally low-pass this combined estimate lightly for state decisions.

Guideline:

- Brake-intent entry/exit should depend on `|v_combined|`, not per-wheel speed deltas.
- Per-wheel speeds remain useful for diagnostics and eventual per-wheel brake balancing, but not for intent trigger alone.

---

## 9) Interaction with existing supervisors and safety

DriverIntentBraking should be a **soft supervisory layer** and must respect:

- existing timeout/failsafe handling,
- stall/current limit supervisors,
- control-mode gating (especially when not in torque-capable mode),
- hard FOC protections and fault shutdown.

Recommended priority order (highest first):

1. Fault/timeout disable logic
2. Mode/safety supervisors
3. DriverIntentBraking
4. Existing command filter/mixer mapping
5. Wheel-level supervisors

This keeps braking intent from fighting hard safety logic.

---

## 10) Parameters for tuning

Define tunables up front:

- `drive_enter_threshold`
- `neutral_deadband`
- `release_rate_threshold`
- `release_to_neutral_window_ms`
- `min_speed_for_brake_intent_rpm`
- `ramp_up_rate`, `ramp_down_rate`
- `tau_up`, `tau_down` (if LPF retained)
- `brake_max`
- `brake_taper_speed_rpm`
- `brake_hold_time_ms`
- offset estimator gains and enable conditions

Expose these similarly to existing user/control parameters for iterative road tuning.

---

## 11) Observability and debug signals

Add telemetry hooks for tuning/validation:

- raw speed command, offset estimate, normalized speed command
- command derivative (`du/dt`)
- combined speed estimate
- intent state enum
- brake intent flag + brake strength
- asymmetric ramp branch active (up/down)

Without these, tuning tends to become trial-and-error and can mask false triggers.

---

## 12) Validation plan (before implementation completion)

Bench + road-test scenarios:

1. **Gentle acceleration**: confirm unchanged ramp-up feel.
2. **Throttle release at speed**: verify fast ramp-down and brake-intent entry.
3. **Steering while cruising**: confirm no false brake trigger from wheel speed split.
4. **Stop approach**: ensure smooth taper without oscillation or reverse jerk.
5. **Noisy neutral joystick**: ensure offset + deadband prevents chatter.
6. **Re-apply throttle during braking**: ensure immediate intent cancellation.
7. **Timeout/fault injection**: ensure safety layers override intent module.

---

## 13) Incremental rollout strategy

1. Integrate module in **observe-only mode** (state + telemetry, no command override).
2. Enable asymmetric ramp-down only.
3. Enable brake-intent trigger with conservative limits.
4. Tune thresholds/time constants on real vehicle.
5. Enable by default behind feature flag after stability confirmation.

This minimizes risk and helps separate logic errors from tuning errors.

---

## 14) Non-goals (for first version)

- No direct changes to generated BLDC controller internals.
- No aggressive ABS-like per-wheel slip regulation in this phase.
- No heavy model-based observer; keep deterministic, lightweight logic suitable for main loop.

