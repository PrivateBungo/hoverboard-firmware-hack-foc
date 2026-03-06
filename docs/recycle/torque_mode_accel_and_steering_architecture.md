# Torque-Mode Acceleration Constraint, Differential-Torque Steering, and Stall-Aware Current Limiting

## Current control pipeline (as implemented)

1. **Input decode + safety + mode arbitration** happens in `readCommand()` / `handleTimeout()`.
2. **Driver-feel shaping** (rate limiter + low-pass) is applied in the main loop to `steer` and `speed` commands.
3. **Differential mixing** maps `(speed, steer)` into `cmdL/cmdR`.
4. `cmdL/cmdR` are turned into motor targets `pwml/pwmr` and passed to each motor model as `r_inpTgt`.
5. The generated BLDC controller executes in one of the modes (`VLT`, `SPD`, `TRQ`) selected by `z_ctrlModReq`.

This already provides a clean seam for adding higher-level wheel command supervision **without changing low-level FOC internals**.

## Existing current-limit/protection behavior (what we should preserve)

- There is a configured motor current limit (`I_MOT_MAX`, currently 15 A in config defaults).
- That value is converted and passed into the generated controller as `i_max`.
- The generated controller also reports error state (`z_errCode`), and the top-level PWM task disables drive if either motor is in error.

Architecturally, this means we should treat `i_max` + `z_errCode` shutdown behavior as the **hard safety envelope**, and add any startup/stall behavior as a **soft supervisor above FOC**, not by weakening hard protections.

## Requirement A: acceleration-constrained torque mode

### Why this fits well

- Today the command shaping is input-based; it does not explicitly limit measured wheel acceleration.
- Measured wheel speed (`rtY_Left.n_mot`, `rtY_Right.n_mot`) is already available in firmware.
- Limiting can be done above FOC (at `cmdL/cmdR` or `pwml/pwmr`) and keep generated controller/protections unchanged.

### Ballpark acceleration targets (based on your vehicle)

Assumptions you gave:
- wheel diameter: **0.4 m** (`r = 0.2 m`)
- vehicle mass: **~50 kg**
- motors: **2 × 400 W peak**

Practical tuning target for jerk reduction on this class of platform:
- **Start at 0.6 m/s² longitudinal acceleration limit**
- Then tune in the range **0.5 to 1.0 m/s²** (comfort to more aggressive)

Wheel angular-acceleration equivalent:
- `alpha = a / r`
- At 0.6 m/s² and r=0.2 m: `alpha = 3.0 rad/s²`
- In motor-speed units: `rpm/s = alpha * 60 / (2π) ≈ 28.6 rpm/s`

Useful limit table:
- 0.5 m/s² → ~23.9 rpm/s
- 0.6 m/s² → ~28.6 rpm/s
- 1.0 m/s² → ~47.7 rpm/s

### Acceleration estimator design (preferred order)

Use **derivative first, then low-pass** (as discussed):

1. Compute raw acceleration from measured speed and explicit timestep variable:
   - `a_raw = (n_now - n_prev) / dt`
2. Apply low-pass to `a_raw`:
   - `a_filt = LPF(a_raw, tau_a, dt)`

Why this order: it makes the estimator behavior explicit with respect to `dt` and avoids burying timing dynamics inside a speed prefilter.

Implementation notes:
- Keep `dt` as a variable (do not hardcode loop time assumptions).
- Use minimum-speed gating / hysteresis at very low speed to avoid hall quantization spikes.

### Recommended architecture

Add a **per-wheel torque supervisor** in main-loop space:

- Inputs: requested wheel torque command + measured wheel speed.
- State: previous speed, filtered acceleration, previous effective torque command.
- Logic:
  1. Estimate acceleration from speed derivative + LPF.
  2. If `|a_filt| > a_max`, reduce effective torque command magnitude.
  3. Use separate limits if needed (`a_max_drive`, `a_max_regen`) plus smooth release.

Keep this as a standalone module (e.g., `torque_supervisor.c/.h`) so `main.c` does not grow further.

## Requirement B: torque-forward steering via differential torque

Steering to a fixed wheel-speed offset is often too weak/indirect on high-grip surfaces and small steering angles. A direct **differential torque bias** is simpler and more effective while staying fully in torque mode.

Use:
- Base drive torque: `T_base` from forward/reverse input
- Steering torque bias: `ΔT_steer` from steering input (and optional gains/scheduling)
- Final commands:
  - `T_right = T_base + ΔT_steer`
  - `T_left  = T_base - ΔT_steer`

This gives direct yaw authority and keeps the drivetrain in pure torque-control semantics.

For your front-axle concept, differential torque is a good match: tire forces generate yaw moment directly, and front mechanics settle to the path naturally. Optional later improvement: speed-scheduled steering gain (reduce `ΔT_steer` as speed rises).

## Requirement C: speed-based soft current limit for startup / stall behavior

### Goal

Avoid repeated trips into hard motor-limit/fault behavior at very low speed or stall attempts by introducing a **soft current envelope** that ramps with speed.

### Requested target behavior

- Hard current limit (safety envelope): **20 A** (future config change)
- Soft current limit at standstill: **10 A**
- Soft limit ramps linearly to hard limit by **7 km/h** wheel speed

### Unit conversion for your wheel size

With wheel diameter `D = 0.4 m`:
- Circumference `C = πD ≈ 1.2566 m`
- `7 km/h = 1.944 m/s`
- Wheel speed at 7 km/h:
  - `rev/s = v/C ≈ 1.547`
  - `rpm ≈ 92.8`

So, use a transition breakpoint around **93 rpm** wheel speed.

### Soft-limit law (per wheel)

Using wheel speed magnitude `n_abs = |n_mot|`:

- `I_soft(n_abs) = I_soft_min + (I_hard - I_soft_min) * clamp(n_abs / n_ramp, 0, 1)`

With requested numbers:
- `I_soft_min = 10 A`
- `I_hard = 20 A`
- `n_ramp = 93 rpm`

Equivalent piecewise form:
- `n_abs <= 0 rpm` → `I_soft = 10 A`
- `0 < n_abs < 93 rpm` → linear 10→20 A
- `n_abs >= 93 rpm` → `I_soft = 20 A`

### Integration strategy

Recommended architecture (no generated-code edits):

1. Keep controller hard limit/fault logic intact.
2. Compute `I_soft` per wheel from measured speed.
3. Convert `I_soft` into a **command clamp scale** for torque request path in `TRQ_MODE`.
4. Apply clamp before writing `pwml/pwmr`.

This behaves like a startup traction governor and reduces stall-triggered panic while preserving hard safety limits.

### Important notes

- If one wheel is near-stalled, per-wheel soft limits avoid overdriving that side.
- Consider a small hysteresis band around zero speed to avoid oscillatory clamp behavior.
- Keep parameters exposed for tuning (`I_soft_min`, `I_hard`, `n_ramp`, hysteresis).

## Suggested structural repo changes (before coding features)

1. **Introduce a drive-control layer** (`Src/drive_control.c`, `Inc/drive_control.h`) between filtering/mixing and `pwml/pwmr` assignment.
2. Move command synthesis out of `main.c` into explicit stages:
   - stage 1: normalized driver intent
   - stage 2: base torque generation
   - stage 3: acceleration supervisor (derivative then LPF, dt-explicit)
   - stage 4: soft current envelope (speed-based)
   - stage 5: steering differential-torque bias
   - stage 6: saturation/sign/output mapping
3. Keep generated BLDC files untouched unless absolutely necessary.
4. Add telemetry for tuning:
   - wheel speeds
   - raw + filtered acceleration estimate
   - acceleration limiter active flag
   - `I_soft`, `I_hard`, soft-limit active flag
   - `T_base`, `ΔT_steer`, final `T_left/T_right`
5. Add feature/mode flag(s):
   - legacy mixer path
   - supervised torque path (accel + soft current + torque-bias steering)

## Implementation order (low risk)

1. Refactor command path into dedicated module with no behavior change.
2. Add acceleration supervisor in `TRQ_MODE`, default disabled.
3. Add speed-based soft current envelope, default disabled.
4. Add differential-torque steering mode behind feature flag.
5. Add parameter exposure + road tuning.

## Risks and mitigations

- **Noisy acceleration estimate at low speed**: derivative + LPF, minimum-speed gating, hysteresis.
- **Clamp chatter near zero speed**: deadband/hysteresis around speed origin.
- **Over-steer from too much torque bias**: clamp `ΔT_steer`, speed-based gain reduction.
- **Mode-transition transients**: reset internal states/integrators when mode changes.
- **Loop-rate mismatch**: keep all formulas dt-explicit; start in main-loop cadence and adjust if testing requires.
