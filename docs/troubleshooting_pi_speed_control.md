# Troubleshooting: Jittery / Unstable PI Speed Control (UART Mode)

This guide covers the most common causes of motor instability, spin-out, or
oscillation when using `VARIANT_USART` (binary serial speed control with
`FEEDBACK_SERIAL_USART2`).

---

## 1. Root-cause taxonomy

| Symptom | Most likely cause |
|---------|-------------------|
| Motor spins out of control the moment the wheel is touched | **Inverted feedback**: measured speed sign is opposite to setpoint sign → PI error grows instead of shrinks |
| Motor oscillates around setpoint; amplitude grows with load | **Gains too high** (KP, KI): PI integrator overwinds, overshoot when disturbance is removed |
| `speedR_meas` in feedback is negative when going forward | **Sign convention mismatch** (pre-correction): raw FOC output for right motor is naturally negative |
| Motor runs at wrong speed in one direction only | **Hall sensor order wrong** on one wheel: measured speed has wrong magnitude or sign for one direction |
| Both motors run but one is always reversed | **Phase wires swapped** on one motor, or `INVERT_L/R_DIRECTION` set incorrectly |
| Motor does not hold against external load; speed drops | **KP / KI too low** or `INT_LIM` too small |

---

## 2. Sign convention overview

The firmware uses a **vehicle-forward** sign convention throughout the PI
control path:

```
speed command (cmd2)  > 0  →  board moves forward
measured speed        > 0  →  board is moving forward
speed error           > 0  →  board is slower than setpoint
PI torque output      > 0  →  increase speed (forward torque)
```

Because the two motors are physically mirrored, the raw FOC output (`n_mot`)
has different natural signs:

| Motor | Default (`n_mot` sign when going forward) |
|-------|------------------------------------------|
| Left  | Positive (no inversion needed) |
| Right | **Negative** (must be negated to get vehicle-forward speed) |

The firmware applies this correction in `calcAvgSpeed()`, in the PI controller
measured-speed path, **and** in the UART feedback frame:

```c
// Default (no INVERT_x_DIRECTION flags):
speedL_vehicle_fwd =  n_mot_left;   // positive when going forward
speedR_vehicle_fwd = -n_mot_right;  // negated: positive when going forward
```

If `INVERT_L_DIRECTION` or `INVERT_R_DIRECTION` are defined, the convention is
flipped for that motor.  If `SPEED_COEFFICIENT` encodes a negative value (MSB
set in the Q14 fixdt representation), both speeds are additionally negated.

---

## 3. Verifying feedback sign correctness

With `VARIANT_USART` and `uart_control_test.py`:

```
python3 tools/scripts/uart_control_test.py --port /dev/ttyUSB0 --speed 100
```

Expected output while ramping to speed=100 (board going forward):

```
[TX] steer=    0  speed=  100
[FB] cmd1=    0  cmd2=  100  speedR=   85  speedL=   88  bat=42.00V  temp=28°C  led=0x0000
```

Both `speedR` and `speedL` should be **positive** and close to the setpoint.

**If `speedR` is negative while `speedL` is positive (or vice versa):**
- The board is wired in the "natural" default configuration and the firmware
  *before* the sign-convention fix was used.  Upgrade to a firmware version
  that includes this fix.
- Alternatively, if running custom firmware: check that the feedback frame
  assembles `fbSpeedRight = -n_mot_right` (not `+n_mot_right`).

**If both speeds are negative when going forward:**
- `SPEED_COEFFICIENT` may be set to a negative value, or both
  `INVERT_L_DIRECTION` and `INVERT_R_DIRECTION` are wrong.

---

## 4. Verifying PI stability with the debug stream

Switch to a variant that has `DEBUG_SERIAL_USART3` enabled (e.g. add
`#define DEBUG_SERIAL_USART3` to your custom config) and connect a USB-UART to
the **right** cable.  Watch the `SetpointTrace` line:

```
SetpointTrace mode:FWD ... vSet:100 aSet:2 vAct:98 vMeasL:97 vMeasR:99 vErr:2 vSat:0 ...
```

Key fields:

| Field | Healthy range | Problem sign |
|-------|---------------|--------------|
| `vSet` | Matches your speed command (after ramp) | Stuck at 0 → calibration inhibit active |
| `vMeasL`, `vMeasR` | Close to `vSet`; same sign | Opposite sign to `vSet` → inverted feedback |
| `vErr` | Small (< 20 rpm) in steady state | Large and growing → gains too high or inverted feedback |
| `vSat` | `0` in normal operation | Persistently `1` → integrator or output saturated |

---

## 5. Common hardware causes

### 5.1 Wrong hall sensor order

Hall sensors on hoverboard motors have no fixed connector order.  If the three
hall wires are connected in the wrong sequence, the FOC controller either:

- Measures speed with incorrect magnitude, or
- Measures speed with the **correct magnitude but wrong sign** for one
  direction.

**How to detect:** `vMeasL` (or `vMeasR`) is positive in one direction but
shows incorrect value in the other direction.  Or `z_errCode` (ErrL/ErrR in
debug) shows bit-1 set (Sum==7 error).

**Fix:** Swap two of the three hall wires on the affected motor and re-flash.

### 5.2 Reversed motor phase wires

If two of the three motor phase wires are swapped, the motor runs in reverse
for the same hall sensor signal.  The firmware `INVERT_x_DIRECTION` flags
correct for this at the software level.

**How to detect:** One wheel spins backward when commanded forward, even after
verifying hall order.

**Fix:** Either physically swap two phase wires, or define `INVERT_L_DIRECTION`
or `INVERT_R_DIRECTION` in your config.

### 5.3 Incorrect `INVERT_x_DIRECTION` settings

If `INVERT_L_DIRECTION` is defined when it should not be (or vice versa):

- `vMeasL` is negative while `vSet` is positive.
- The PI error is `vSet - vMeasL`, which is now **larger** than the setpoint
  instead of smaller as speed increases → positive feedback → runaway.

**Fix:** Toggle the `INVERT_L_DIRECTION` / `INVERT_R_DIRECTION` define.

---

## 6. PI gain tuning

Default gains (see `config/control_tuning/motor_controller_gains.h`):

| Parameter | Default | Units |
|-----------|---------|-------|
| `MOTOR_CTRL_VEL_KP_Q15` | 13107 ≈ 0.40 | (torque cmd) / rpm |
| `MOTOR_CTRL_VEL_KI_Q15` | 655 ≈ 0.02 | (torque cmd) / rpm / loop |
| `MOTOR_CTRL_INT_LIM` | 400 | torque cmd units |
| `MOTOR_CTRL_TORQUE_MAX` | 1000 | torque cmd units |

### 6.1 Tuning procedure

1. Start with defaults.  If the motor holds its speed against a gentle hand
   load, the loop is stable.
2. If response is **too slow** (setpoint not reached, large steady-state
   error): increase `MOTOR_CTRL_VEL_KP_Q15` in steps of ~6554 (≈0.20).
3. If the motor **oscillates** or overshoots badly after a disturbance:
   - First reduce `MOTOR_CTRL_VEL_KI_Q15` by half.
   - If still oscillating, reduce `MOTOR_CTRL_VEL_KP_Q15` by 25%.
4. `MOTOR_CTRL_INT_LIM` limits how much the integrator can accumulate.  Set
   it to less than `MOTOR_CTRL_TORQUE_MAX` (e.g. 400–600) so the integrator
   alone cannot saturate the output.

### 6.2 Why the defaults were chosen conservatively

The default gains prioritise **stability** over response speed.  Hoverboard
motors have relatively high inertia and low damping; with the previous defaults
(KP≈0.80, KI≈0.05 per loop) a sudden hand-load on a spinning wheel could
drive the integrator to near-maximum within one second, then produce a large
overshoot torque spike when the load was removed.

---

## 7. Checklist for "motor spins out of control when wheel is touched"

Work through these checks in order:

- [ ] **1. Check feedback sign** — With `uart_control_test.py`, confirm that
  `speedR` and `speedL` are both positive when going forward.  If not, the
  firmware may be missing the sign-convention fix.

- [ ] **2. Check `vMeasL` / `vMeasR` in SetpointTrace** — Connect debug UART
  and verify that `vMeasL` and `vMeasR` are in the same sign as `vSet`.  A
  sign mismatch means the PI loop has positive feedback → runaway.

- [ ] **3. Verify `INVERT_x_DIRECTION` flags** — Spin each motor slowly by
  hand in the forward direction and observe `n_mot` in debug.  If `n_mot_left`
  is negative, define `INVERT_L_DIRECTION`.  If `n_mot_right` is positive,
  define `INVERT_R_DIRECTION`.

- [ ] **4. Verify hall sensor order** — If `z_errCode` bit-1 is set, hall
  order is likely wrong.  Swap hall wires (two at a time) and retry.

- [ ] **5. Reduce PI gains** — If the above checks pass but the motor still
  oscillates under load, lower `MOTOR_CTRL_VEL_KP_Q15` and
  `MOTOR_CTRL_VEL_KI_Q15` as described in §6.

- [ ] **6. Check `SPEED_COEFFICIENT`** — If set to a negative value, both
  motor speeds are additionally inverted.  Verify the sign of this coefficient
  matches your physical setup.

---

## 8. Quick sign-convention reference

```
calcAvgSpeed() (util.c):
  speedAvg  = +n_mot_left  (default, no INVERT_L)
  speedAvg -= +n_mot_right (default, no INVERT_R → subtract because mirrored)
  speedAvg /= 2
  if (SPEED_COEFFICIENT & (1 << 15)): speedAvg = -speedAvg  // bit 15 = sign bit of 16-bit fixdt(1,16,14) container

PI controller (main.c):
  measuredSpeedLeft  = +n_mot_left   (no INVERT_L)
  measuredSpeedRight = -n_mot_right  (no INVERT_R → negate)
  if (SPEED_COEFFICIENT & (1 << 15)): both negated

Feedback frame (main.c → UART host):
  speedL_meas = measuredSpeedLeft   (vehicle-forward, same as PI)
  speedR_meas = measuredSpeedRight  (vehicle-forward, same as PI)
```

All three use the same convention: **positive = vehicle forward**.  A sign
mismatch between any two of these is a root cause of instability.
