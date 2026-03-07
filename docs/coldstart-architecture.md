# Cold-Start Architecture: Open-Loop Sinusoidal Startup

## 1. The Problem — Electromagnetic Lock at Standstill

### Why FOC Locks at Standstill

The firmware uses **Field-Oriented Control (FOC)** for efficient, smooth motor drive. FOC depends on knowing the rotor's instantaneous electrical angle so it can align the stator magnetic field at 90° to the rotor flux (maximum torque per ampere). This angle is computed by the BLDC controller's hall-interpolation block (`n_commDeacv` relay in `BLDC_controller.c`).

The problem arises because the interpolation relay is **disabled at low speed**:

```
n_commDeacv_Mode = true   when  |speed| >= n_commDeacvHi (480 = 30 rpm)
n_commDeacv_Mode = false  when  |speed| <  n_commAcvLo   (240 = 15 rpm)
```

Below 30 rpm (or at standstill), `n_commDeacv_Mode == false`, and the controller falls back to **`COM_Method`** — 6-step block commutation — which looks up a pre-defined voltage vector from `z_commutMap_M1_table` based on the current hall sector.

### The Deadlock Signal Chain

At standstill with a non-zero torque command, the signal chain is:

1. **Frozen angle** — no hall transitions → position estimator stays at last sector centre → electrical angle is fixed.
2. **Static voltage vector** — `COM_Method` applies the commutation map for the current sector → voltage is applied to fixed phase pair (like a stepper motor in hold mode).
3. **No rotation** — the static field creates a holding torque in a fixed direction. If the rotor is already at or near that position, the electromagnetic holding force prevents movement.
4. **PI windup** — the torque (or current) PI controller in `F05_FOC` keeps integrating the error. Because the motor cannot move, the setpoint is never met → integrator saturates.
5. **Maximum holding current** — PI output clamps to maximum → maximum current into the static phase pattern → stronger electromagnetic lock.
6. **No hall transitions** — because the rotor cannot move, hall sensors never change state → speed estimate stays at zero → `n_commDeacv_Mode` never becomes true → the system is permanently stuck.

**The harder you push the torque command, the harder the motor locks.** This is an inherent property of 6-step commutation with a PI controller that does not have anti-windup for zero-speed conditions.

---

## 2. The Solution — Open-Loop Sinusoidal Startup

### Overview

The open-loop startup bypasses the BLDC controller's output at standstill and feeds a **rotating sinusoidal magnetic field** directly to the motor phases. The rotor follows this field like a stepper motor following a step sequence, building up speed until the FOC interpolation activates naturally.

The implementation (`OPENLOOP_ENABLE` in `config.h`, logic in `Src/bldc.c`) inserts between the BLDC controller step and the timer register writes. The BLDC controller **keeps running** (tracking halls, building its speed estimate) while open-loop overrides the phase outputs.

### Two-Phase Sequence

#### Phase 1: ALIGN (0 → `OPENLOOP_ALIGN_DURATION` cycles, default 0.2 sec)

1. Read the current hall sector from the hall sensor GPIO pins.
2. Map the hall value to a sector number using `rtConstP.vec_hallToPos_Value[hallCode]`.
3. Compute the starting electrical angle as the **centre of the hall sector**: `theta = sector × 3840 + 1920`.
4. Ramp the output voltage from 0 to `OPENLOOP_VOLTAGE_MAX` over the align duration.
5. Hold `theta` **constant** — no rotation yet.

Result: the rotor gently snaps to the nearest known magnetic position without a sudden current surge. The gradual ramp avoids overcurrent and mechanical jerk.

#### Phase 2: ROTATE (align end → FOC handoff)

1. Begin advancing `theta` each ISR cycle: `theta += delta_theta`.
2. `delta_theta` ramps from 0 to `OPENLOOP_DELTA_THETA_MAX` over `OPENLOOP_ACCEL_DURATION` cycles.
3. Voltage amplitude stays constant at `OPENLOOP_VOLTAGE_MAX`.
4. Direction follows the sign of `pwml` / `pwmr` (input command).

The rotating field drags the rotor like a stepper motor. As the rotor accelerates:
- Hall sensor transitions begin occurring.
- The BLDC controller builds a speed estimate.
- Once `|speed| >= n_commDeacvHi` (30 rpm), `n_commDeacv_Mode` flips to `true`.

### Handoff to FOC

When `rtDW_Left.n_commDeacv_Mode == true` (or right), the open-loop logic **stops overriding** the phase outputs. On the very next ISR cycle, the BLDC controller's FOC output flows through unmodified. A small transient jerk at handoff is acceptable; no blending is required because the BLDC controller has been tracking hall positions throughout the open-loop sequence.

The open-loop state is reset to `phase=0` on exit, so a subsequent standstill will trigger a new startup sequence.

---

## 3. The Angle System

### Units and Relationships

| Quantity | Value |
|---|---|
| One electrical revolution | 23040 angle units (360° × 64) |
| Motor pole pairs | 15 (default for hoverboard hub motors) |
| One mechanical revolution | 15 electrical revolutions = 345600 angle units |
| Hall sectors per electrical revolution | 6 |
| Angle units per hall sector | 3840 |
| Sector centre offset | 1920 (half a sector) |

### Calculating Δθ for a Target RPM

```
Δθ = (target_mech_RPM × pole_pairs × angle_per_elec_rev) / (60 × ISR_freq)
   = (target_mech_RPM × 15 × 23040) / (60 × 16000)
```

Example values:

| Target (mech RPM) | Δθ per cycle |
|---|---|
| 10 | ≈ 3.6 |
| 40 | ≈ 14.4 |
| 100 | ≈ 36 |

### Speed Thresholds (fixed-point `fixdt(1,16,4)`)

| Parameter | Raw value | Actual RPM |
|---|---|---|
| `n_commDeacvHi` (FOC activates) | 480 | 30 rpm |
| `n_commAcvLo` (6-step reactivates) | 240 | 15 rpm |
| `n_stdStillDet` (standstill gate, default 0) | 0 | 0 rpm |

The relay has **hysteresis**: FOC activates at 30 rpm but does not deactivate until speed falls below 15 rpm.

---

## 4. Tuning Guide

| Symptom | Cause | Fix |
|---|---|---|
| Motor stalls / rotor doesn't move | Voltage too low to overcome friction or load | Increase `OPENLOOP_VOLTAGE_MAX` (e.g. 900–1200) |
| Motor vibrates / buzzes but doesn't rotate smoothly | Voltage too high, rotor overshoots each step | Decrease `OPENLOOP_VOLTAGE_MAX` or increase `OPENLOOP_ACCEL_DURATION` |
| Motor loses sync (skips poles, stutters) | Acceleration too fast | Decrease `OPENLOOP_DELTA_THETA_MAX` (lower top speed) or increase `OPENLOOP_ACCEL_DURATION` |
| Handoff jerk is harsh | Angular velocity mismatch at handoff | Lower `OPENLOOP_DELTA_THETA_MAX` so handoff speed is closer to `n_commDeacvHi` |
| Motor doesn't start until large command | Deadband too large | Decrease the deadband threshold (currently 50 raw counts) in `bldc.c` |
| Open-loop triggers on noise | Deadband too small | Increase the deadband threshold |

The default parameters are conservative: 700 amplitude (~5% duty cycle), 0.2 sec align, 1.0 sec ramp to 100 RPM. These should work on most hoverboard hub motors under no-load conditions.

---

## 5. Code Location

| File | Purpose |
|---|---|
| `Src/bldc.c` | Open-loop state machine, sinusoidal generation, output override |
| `Inc/config.h` | `OPENLOOP_ENABLE`, `OPENLOOP_VOLTAGE_MAX`, timing and speed parameters |
| `Src/BLDC_controller.c` | BLDC FOC controller (auto-generated, not modified) |
| `Inc/BLDC_controller.h` | `DW` struct (contains `n_commDeacv_Mode`), `ConstP` struct (sin tables) |

### Key Identifiers in `bldc.c`

```c
/* State structs — one per motor */
static OpenLoopState olStateL;   /* left motor */
static OpenLoopState olStateR;   /* right motor */

/* Entry: phase==0, enableFin, |pwm|>50, !n_commDeacv_Mode */
/* Exit:  n_commDeacv_Mode==true  OR  enableFin==0  OR  |pwm|<=50  */
```

### How It Interacts With the BLDC Controller

```
ISR loop (16 kHz):
  1. Read hall sensors
  2. Set rtU_Left / rtU_Right inputs
  3. BLDC_controller_step()        <-- FOC runs every cycle, builds speed estimate
  4. Read rtY_Left.DC_phaA/B/C    <-- default outputs
  5. [OPEN-LOOP BLOCK]            <-- override outputs if active
  6. Write timer registers
```

The open-loop block reads `rtDW_Left.n_commDeacv_Mode` (set inside step 3) and acts on it in step 5 of the same ISR cycle.

---

## 6. The Deadlock Mechanism Explained

Understanding *why* high torque commands make the lock worse is key to appreciating the open-loop solution.

The BLDC FOC controller runs in `F05_FOC` every ISR cycle regardless of commutation method. With `n_commDeacv_Mode == false` (6-step mode), the current loop PI controllers still execute:

1. The **Iq PI controller** (`cf_iqKp`, `cf_iqKi`) computes: `V_q_cmd = Kp × e_q + Ki × ∫e_q dt`  where `e_q = Iq_setpoint − Iq_measured`.

2. At standstill, `Iq_measured ≈ 0` (no rotation → no back-EMF → no sustained current in the motor inductance). Therefore `e_q ≈ Iq_setpoint` and the integrator **winds up continuously**.

3. The PI output saturates at `V_q_max`. This saturated voltage is then routed through `COM_Method` which maps it to fixed phase voltages using the commutation map for the current hall sector.

4. Maximum voltage into a fixed direction → **maximum electromagnetic holding torque** opposing rotor displacement. The motor acts like a stepper motor at maximum holding current.

5. No displacement → no hall transitions → speed estimate stays zero → `n_commDeacv_Mode` stays false → same commutation sector → same holding force → **deadlock is stable**.

The open-loop startup **breaks this cycle** by temporarily taking over the PWM outputs and providing a rotating field that can produce **net rotational force**, bypassing both the PI integrator path and the static commutation map.

---

## 7. Future Improvements

- **Angle blending at handoff**: Instead of an abrupt switch, interpolate between the open-loop synthetic angle and the FOC estimated angle over a short window (e.g. 200 cycles) to eliminate handoff jerk entirely.
- **Current limiting during open-loop**: Monitor `curL_DC` / `curR_DC` and reduce voltage if DC link current exceeds a threshold, providing active overcurrent protection independent of the BLDC controller.
- **Adaptive voltage**: Use DC link current feedback to automatically adjust `voltage` during the rotate phase, targeting a specific holding current rather than a fixed voltage amplitude.
- **Sensorless transition**: At higher speeds, transition from hall-based FOC to a back-EMF sensorless estimator, eliminating dependence on hall sensor signal quality at high speed.
- **Anti-windup reset**: On open-loop exit, reset the PI integrator states in `rtDW_Left` and `rtDW_Right` to prevent the accumulated windup from causing a current spike at FOC handoff.
