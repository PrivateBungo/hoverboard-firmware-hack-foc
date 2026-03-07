# Motor Startup Architecture

*Last updated: 2026-03-07  
CSV data sources: debug_20260307_215508.csv (pre-fix), debug_20260307_222136.csv (with OPENLOOP_ENABLE)*

---

## 1. Ideal Startup Architecture

A well-behaved sensor-commutated PMSM startup without an absolute encoder needs three distinct phases.

### Phase 1 — Field lock (rotor position capture)

The controller applies a **static sinusoidal voltage vector** to the motor phases for long enough that the rotor rotates to, and locks at, the magnetic equilibrium of that field. The rotor's mechanical position is now known: it equals the field angle. Duration should be long enough that even a heavily-loaded or high-inertia rotor reaches equilibrium; 100–300 ms is typical for hub motors.

Key property: the voltage must ramp up from zero to avoid a current spike and mechanical jerk. Final voltage needs to be large enough to overcome static friction and hold the rotor firmly against small disturbances.

### Phase 2 — Controlled acceleration (open-loop)

The controller advances the field angle **smoothly and slowly** at a rate the rotor can follow:

- Initial Δθ/cycle = 0 (matching field angle = rotor angle at end of Phase 1).
- Δθ/cycle ramps up over one to two seconds to a target speed that is comfortably above the FOC activation threshold.
- The rotor follows like a synchronous motor tracking a stepper sequence.
- Hall transitions begin appearing; the BLDC controller accumulates a speed estimate.

Key property: acceleration must be slow enough that the rotor never loses synchronisation with the field (i.e. electromagnetic torque always exceeds load torque plus inertia torque during the ramp).

### Phase 3 — FOC handoff

Once the speed estimate exceeds the FOC-activation threshold, the BLDC controller's Hall-interpolated angle estimator is trusted and closed-loop FOC takes over:

- The `n_commDeacv_Mode` relay flips to `true` (speed ≥ `n_commDeacvHi`).
- The open-loop output override is released.
- The BLDC controller's Park/Clarke + torque PI path drives the motor directly.
- If speed later falls below `n_commAcvLo`, the system reverts to 6-step commutation (not back to open-loop startup; that re-engages only when the motor is stopped and re-commanded).

### What must NOT happen

| Condition | Why it matters |
|---|---|
| Static voltage applied to a stalled motor | Creates electromagnetic lock — the harder the command, the harder the lock |
| PI integrators running while phases are overridden | Integrators wind up unchecked; at FOC handoff the wound-up state creates a current spike |
| Speed estimate below threshold when FOC is needed | Prevents the controller from ever leaving 6-step / open-loop |
| Diagnostic latch (errCode=4) firing before rotor moves | Permanently blocks re-entry to TRQ_MODE; motor runs in zero-output OPEN_MODE forever |

---

## 2. Current Implementation

### 2.1 ISR execution order

```
16 kHz ISR:
  1. Read hall GPIOs → rtU_Left / rtU_Right inputs
  2. BLDC_controller_step()           ← entire FOC, hall tracking, diagnostics
  3. Read rtY_Left.DC_phaA/B/C        ← BLDC controller's phase commands
  4. [OPENLOOP_ENABLE block]          ← may override (3) with sinusoidal output
  5. Write computed ul/vl/wl to PWM timer registers
```

Note: `rtY_Left.DC_phaA/B/C` (step 3) is what the debug CSV logs as `phAL/phBL/phCL`.
The **actual motor drive** is `ul/vl/wl` written in step 5, which may differ when open-loop overrides.

### 2.2 BLDC controller subsystems relevant to startup

**F02_Diagnostics** (runs every cycle, `b_diagEna=1`):

```
errCode = (Hall==000)<<0 | (Hall==111)<<1 | standstillLatch<<2

standstillLatch:
  - new trigger:  b_motEna  &&  Abs5 < n_stdStillDet  &&  |PI_output| > r_errInpTgtThres
  - latch:        (UnitDelay_DSTATE_e & 4) != 0  →  latch stays true forever

Debounce_Filter(errCode!=0, t_errQual=1280, t_errDequal=9600)  →  Merge_p
```

`t_errQual=1280 cycles = 80 ms` to qualify; `t_errDequal=9600 cycles = 600 ms` to clear.
Once the standstill latch fires, `errCode` always has bit 2 set → `Merge_p` is permanently `true`.

**F03_Control_Mode_Manager** (runs every cycle):

```
rtb_RelationalOperator1_mv = Merge_p  ||  !b_motEna  ||  (z_ctrlModReq==0)

if rtb_RelationalOperator1_mv:
    z_ctrlMod = OPEN_MODE  (zero-output)        ← controller cannot exit this
else:
    z_ctrlMod follows z_ctrlModReq (TRQ_MODE etc.)
```

Once `Merge_p=true`, `rtb_RelationalOperator1_mv` is always true → motor is permanently in the BLDC OPEN_MODE (zero phase output from the BLDC controller).

**F04_Hall_Estimation** (runs every cycle regardless of z_ctrlMod):

```
n_commDeacv relay:
  Abs5 >= n_commDeacvHi  →  n_commDeacv_Mode = true   (FOC active)
  Abs5 <= n_commAcvLo    →  n_commDeacv_Mode = false  (6-step active)

rtb_LogicalOperator = n_commDeacv_Mode && !dz_cntTrnsDet
```

`n_commDeacv_Mode` controls which commutation path is used (FOC vs 6-step) AND controls whether `bldc.c` OPENLOOP exits. This subsystem still updates speed even when `z_ctrlMod=OPEN_MODE`.

### 2.3 Parameters set in `BLDC_Init()` (Src/util.c)

| Parameter | Value set | Meaning |
|---|---|---|
| `n_stdStillDet` | 0 | Disable standstill latch trigger (Abs5 < 0 is impossible) |
| `n_commDeacvHi` | `2 << 4` = 32 | FOC activates at 2 rpm |
| `n_commAcvLo` | `1 << 4` = 16 | 6-step reactivates at 1 rpm |
| `dz_cntTrnsDetHi` | `z_maxCntRst + 2` = 2002 | Disables transition-detection relay (Counter() saturates at 2001) |
| `Switch2_e` | 1 (both motors) | Correct initial angle estimator state (prevents 60° offset on first edge) |
| `N_MOT_MAX` | 550 rpm | Headroom for typical hoverboard wheel speed |

### 2.4 `OPENLOOP_ENABLE` state machine (Src/bldc.c)

```
Entry condition (each ISR):
  phase==0 && enableFin && |pwm|>50 && !n_commDeacv_Mode

Phase 1 — ALIGN (OPENLOOP_ALIGN_DURATION=3200 cycles = 0.2 s):
  theta  = sector_centre (from current hall, held constant)
  voltage ramps 0 → OPENLOOP_VOLTAGE_MAX (700)

Phase 2 — ROTATE (OPENLOOP_ACCEL_DURATION=16000 cycles = 1.0 s ramp):
  delta_theta ramps 0 → OPENLOOP_DELTA_THETA_MAX (36 units/cycle ≈ 100 rpm mech.)
  theta advances each cycle
  voltage = OPENLOOP_VOLTAGE_MAX (constant)

Exit condition (any cycle while phase>0):
  !enableFin  ||  |pwm|<=50  ||  n_commDeacv_Mode==true
  → resets phase=0, voltage=0, delta_theta=0
```

The override writes directly to `ul/vl/wl` (timer inputs), bypassing `rtY_Left.DC_phaA/B/C`.

---

## 3. Observed Behavior

*Analysis from debug_20260307_222136.csv (firmware with `OPENLOOP_ENABLE`, `n_stdStillDet` not yet fixed)*

### Timeline trace

| t (ms) | Event | CSV evidence |
|---|---|---|
| 114500 | Board running, motor idle | cmdL=6, errL=0, hallL=6, spdL=0 |
| ~115000 | cmdL exceeds 50 → OPENLOOP Phase 1 starts | cmdL=58 first exceeds threshold |
| 115250 | PI windup: phBL=900 (BLDC controller at max output) | `phBL=900` in CSV; OPENLOOP is overriding this with its sinusoidal output, but the BLDC controller has already wound up its PI |
| 115375 | **errL=4 fires** — standstill latch triggers | `errL` column changes from 0 to 4; `iqL=-933, idL=-1024` (≈29A computed) |
| ~115455 | Merge_p goes true after 80 ms debounce | F03 switches BLDC z_ctrlMod to OPEN_MODE; BLDC controller now outputs zero from rtY |
| 115375–118375 | Motor not yet rotating | hallL=6, spdL=0, phAL/B/C all 0 (BLDC zero-output); but cABL/cBCL ≈ −270 to −450 ADC counts (actual motor current from OPENLOOP in bldc.c) |
| **118500** | **First hall change** | hallL: 6→4→3→2→... Motor breaks free from static friction |
| 118500–119750 | Motor moves chaotically, both directions | spdL: −19, −34, −32, −19, +10, +44, +80, +57, +20; hall sectors jump non-sequentially |
| 120000–122750 | Motor settles in oscillation around sector 6 | hallL≈6, angL≈300–360, spdL small and noisy; no smooth rotation; errL=4 throughout |

### Key discrepancy: CSV phase columns vs. actual motor drive

`phAL/phBL/phCL` in the CSV are `rtY_Left.DC_phaA/B/C` — the BLDC **controller's** computed output.
The **motor actually receives** `ul/vl/wl` from bldc.c, which OPENLOOP has overridden.
This means:
- "phBL=900" (row 7) reflects BLDC PI windup, not actual motor voltage.
- "phAL=phBL=phCL=0" (rows 13–32) means the BLDC controller outputs zero, but cABL/cBCL ≠ 0 because bldc.c OPENLOOP is still driving the motor with its sinusoidal output.

### What the motor actually receives after errL=4

```
bldc.c OPENLOOP:     theta advances at OPENLOOP_DELTA_THETA_MAX sinusoidal → actual motor drive
BLDC controller:     z_ctrlMod=OPEN_MODE → DC_phaA/B/C = 0 (overridden by bldc.c anyway)
Net motor drive:     controlled by bldc.c OPENLOOP sinusoidal only
```

### Oscillation pattern after motor breaks free

After the rotor starts moving (~118500), halls change rapidly but non-sequentially (4→3→2→4→1→1→4→6→1→5→4→...). When speed briefly exceeds `n_commDeacvHi` (2 rpm):

1. `n_commDeacv_Mode` → true
2. bldc.c OPENLOOP exits (phase=0, outputs zero)
3. BLDC controller takes over: but `z_ctrlMod=OPEN_MODE` (Merge_p still true) → outputs zero
4. Motor decelerates; `n_commDeacv_Mode` → false
5. bldc.c OPENLOOP re-enters Phase 1 (or directly Phase 2)
6. Back to step 1

This creates a **start-stop oscillation loop**: OPENLOOP drives motor → BLDC controller tries to take over but is locked in OPEN_MODE → motor dies → OPENLOOP restarts.

---

## 4. Hypotheses

### H1 — errCode=4 is the primary failure mode (high confidence)

**Claim**: The standstill latch (`errCode` bit 2) fires because `n_stdStillDet=48` (3 rpm default was not overridden before the fix) while the BLDC PI is wound up at stall. This permanently forces the BLDC controller into OPEN_MODE, breaking the bldc.c OPENLOOP ↔ FOC handoff cycle.

**Supporting evidence**:
- `errL=0` at startup, becomes `4` within 875 ms in both CSVs before the fix
- errL=4 fires at `iqL=1455` / `iqL=-933` (≈29A computed), consistent with PI saturation
- After errL=4, BLDC controller outputs zero (`phAL=phBL=phCL=0`) while actual current (cABL/cBCL) remains non-zero
- Motor oscillates rather than spinning up smoothly, exactly as predicted by the OPENLOOP↔BLDC-zero-output loop described above

**Predicted fix**: Setting `n_stdStillDet=0` makes Abs5 < 0 impossible → latch can never fire → `Merge_p` stays false → BLDC controller can exit OPEN_MODE normally once `n_commDeacv_Mode` becomes true.

**How to verify**: Flash the PR (n_stdStillDet=0), capture CSV. `errL` should remain 0 throughout startup.

---

### H2 — PI integrators wind up during OPENLOOP phase, causing a current spike at FOC handoff (medium confidence)

**Claim**: While bldc.c OPENLOOP overrides the phase outputs, the BLDC controller's torque PI (`cf_iqKp`, `cf_iqKi`) keeps running. It sees `i_phaAB`, `i_phaBC` from ADC (which reflects the OPENLOOP sinusoidal current) and a torque setpoint from `cmdL`. These may not be consistent, causing the integrator to drift. When OPENLOOP exits and FOC takes over the next ISR cycle, the wound-up integrator produces a current spike.

**Supporting evidence**: Indirect — the abrupt phase current jumps visible in cABL/cBCL at the transition point (e.g. at t=118500: cABL jumps from −270 to −1154 in one 125 ms window).

**How to verify**:
1. Add `rtDW_Left.Integrator_DSTATE` (the torque PI state) to the CSV output immediately before and after a FOC handoff.
2. Alternatively, watch `iqL` and `dcL` at the exact transition row: if `dcL` spikes beyond `I_DC_MAX` and then declines over 1–2 rows, windup is the cause.
3. Compare the handoff behaviour at two different `OPENLOOP_VOLTAGE_MAX` values: a lower voltage creates less torque-PI error → less windup → smaller handoff spike.

---

### H3 — Open-loop voltage is at the edge of stiction threshold (medium confidence)

**Claim**: `OPENLOOP_VOLTAGE_MAX=700` (~4.3% duty cycle) is barely enough to overcome static friction on this particular wheel under load. This explains the 3-second delay (rows 8–33 span ~3 seconds) between OPENLOOP starting and the first hall transition.

**Supporting evidence**: OPENLOOP ACCEL_DURATION=16000 cycles (1.0 s). By t≈116200 (1.2 s after OPENLOOP start at t≈115000), the open-loop should be at full 100 rpm rate. Yet the first hall change doesn't appear until t=118500 (~3.5 s after OPENLOOP start). The motor is trying but not breaking free for 2.5 extra seconds.

**How to verify**:
1. Increase `OPENLOOP_VOLTAGE_MAX` to 1000, 1200 in steps. If the first hall transition moves earlier, stiction is the bottleneck.
2. Run the test on an unloaded wheel (lifted off the ground). If the motor starts cleanly, the wheel's ground load (deformation, bearing preload) is the cause.
3. Add `olStateL.phase` and `olStateL.theta` to the CSV output (requires uncommenting debug fields or adding new printf fields). Watch whether theta advances smoothly for the full expected duration before the rotor breaks free.

---

### H4 — Commutation direction inconsistency at FOC handoff (low-medium confidence)

**Claim**: When bldc.c OPENLOOP exits and FOC takes over, the FOC angle estimate jumps from the BLDC controller's hall-sector-centre value to the interpolated value. If `Switch2_e` was 0 (the pre-fix default), the first FOC-driven cycle applies a torque at 60° electrical away from the correct field angle, causing a mechanical jerk and possibly Hall-state confusion.

**Supporting evidence**: The pre-fix data shows spdL = −42 at the bang moment (first CSV, row 8), indicating the motor lurched backward rather than forward, consistent with a 60° angle error flipping the effective torque direction. The fix sets `Switch2_e=1` to correct this.

**How to verify**: Compare CSV data before and after the `Switch2_e=1` fix. If the first measured `spdL` after OPENLOOP exit is positive (matching cmdL sign), the direction is correct.

---

### H5 — OPENLOOP acceleration rate is too high relative to rotor inertia + load (low confidence)

**Claim**: `OPENLOOP_DELTA_THETA_MAX=36` corresponds to 100 rpm mechanical. If the rotor cannot be accelerated to 100 rpm within the ramp window, it loses synchronisation with the field. This would cause the field to "lap" the rotor and produce alternating forward/backward torque — exactly the chaotic hall-sector hopping seen at t=118500–120000.

**Supporting evidence**: The hall sectors after first motion jump non-sequentially (4→3→2→4→1→1→4...) rather than monotonically (6→5→4→3→2→1→6). The measured spdL changes sign (−19, −34, +10, +44, +80, +57) inconsistently.

**How to verify**:
1. Reduce `OPENLOOP_DELTA_THETA_MAX` from 36 to 14 (≈40 rpm) and `OPENLOOP_ACCEL_DURATION` proportionally. Capture CSV. If hall sectors now advance monotonically, sync was being lost.
2. After fixing H1 (errCode=4), re-run with default parameters. If H1 was masking H5, the oscillation may persist even with the errCode fix.

---

## 5. Follow-Up Experiments

The experiments below are listed in priority order. Each one tests a specific hypothesis with a clear pass/fail criterion, rather than just "tweak X and hope."

### Exp-1 — Verify errCode=4 is gone with n_stdStillDet=0

**What**: Flash the PR firmware. Run the test script. Capture CSV.

**Pass criterion**: `errL` column stays 0 throughout the entire startup sequence.  
**Fail criterion**: `errL` becomes 4 → the latch fires through a different path (perhaps a Hall error code bit 0 or 1, meaning the motor was in a Hall-invalid state).

**Why first**: This is the highest-confidence fix. If it works, hypotheses H2–H5 can be tested cleanly.

---

### Exp-2 — Add internal state to CSV (instrument the firmware)

**What**: Add three columns to the CSV printf:
1. `olPhL` — `olStateL.phase` (0=idle, 1=align, 2=rotate)  
2. `olThL` — `olStateL.theta` (synthetic angle)  
3. `fMod` — `rtDW_Left.z_ctrlMod` (0=OPEN, 1=VLT, 2=SPD, 3=TRQ)

**Pass criterion**: CSV clearly shows phase transitions (0→1→2→0) and z_ctrlMod going 0→3 (TRQ_MODE) cleanly at FOC handoff without bouncing.  
**Fail criterion**: z_ctrlMod bounces back to 0 after handoff → confirms BLDC OPEN_MODE is being re-entered after handoff.

**Why**: Eliminates ambiguity in all future data analysis. Without knowing `olStateL.phase`, `z_ctrlMod`, and `theta`, root-cause analysis requires inference.

---

### Exp-3 — Measure PI integrator state at handoff

**What**: Add the torque PI integrator state to the CSV. In BLDC_controller.c, the Iq integrator UnitDelay is `rtDW_Left.Integrator_DSTATE` (find exact name with `grep Integrator_DSTATE Src/BLDC_controller.c`). Log it at 125 ms intervals.

**Pass criterion**: Integrator value at the moment of OPENLOOP exit (fMod transition 0→3) is small (within ±500 counts).  
**Fail criterion**: Integrator is large (thousands of counts) at handoff → confirms H2, anti-windup fix needed.

---

### Exp-4 — Test OPENLOOP_VOLTAGE_MAX effect on time-to-first-hall-transition

**What**: Run three back-to-back tests with `OPENLOOP_VOLTAGE_MAX` = 700, 1000, 1400. Measure the number of CSV rows between `errL` clearing (if Exp-1 confirms fix) and the first `hallL` change.

**Pass criterion (H3 falsified)**: Time-to-first-transition is similar at all voltages → stiction is not the bottleneck; the motor was simply waiting for the field to reach the rotor's locked position.  
**Pass criterion (H3 confirmed)**: Time-to-first-transition decreases with higher voltage → stiction is the bottleneck; raise OPENLOOP_VOLTAGE_MAX.

---

### Exp-5 — Check hall-sequence monotonicity during OPENLOOP

**What**: After fixing Exp-1 (errCode=4 gone) and adding Exp-2 instrumentation, capture a fresh CSV. Extract the `hallL` column from the first OPENLOOP Phase 2 row to the FOC handoff row. Count how many hall transitions are "in order" (e.g. 6→5→4) vs. "reversed" (5→6 while advancing forward).

**Pass criterion (H5 falsified)**: ≥90% of transitions are in the expected direction → OPENLOOP speed is well within capability.  
**Fail criterion (H5 confirmed)**: Many reversed transitions → motor losing synchronisation with the field; reduce `OPENLOOP_DELTA_THETA_MAX`.

---

### Exp-6 — Validate FOC steady-state after full startup

**What**: After all startup issues are resolved (Exp-1 passing), let the motor run at `cmdL=100` for 10 seconds. Inspect `spdL`, `iqL`, `idL`, `hallL`, `angL` during steady-state FOC.

**Pass criterion**: `spdL` is stable within ±10 rpm of expected; `hallL` advances sequentially; `angL` advances smoothly (not jumping in 60° steps); `idL` ≈ 0 (no unnecessary d-axis current).  
**Fail criterion**: `spdL` oscillates badly → PI gains too aggressive for this motor's inertia; `hallL` still shows non-sequential jumps → Hall wiring or direction issue.

---

## 6. Code Location Reference

| File | Purpose |
|---|---|
| `Src/bldc.c` | OPENLOOP state machine, sinusoidal output generation, override logic |
| `Inc/config.h` | `OPENLOOP_ENABLE`, `OPENLOOP_VOLTAGE_MAX`, alignment and ramp timing |
| `Src/util.c` | `BLDC_Init()` — runtime parameter overrides for startup tuning |
| `Src/BLDC_controller.c` | Auto-generated FOC: F02 Diagnostics (errCode), F03 Mode Manager, F04 Hall Estimation, F05 PI controllers |
| `Src/BLDC_controller_data.c` | Default parameter values (overridden by BLDC_Init where noted) |
| `Inc/BLDC_controller.h` | `DW` struct (contains `n_commDeacv_Mode`, `Switch2_e`, PI states), `P` struct (tunable parameters) |

### Fixed-point speed thresholds (`fixdt(1,16,4)`, divide by 16 to get rpm)

| Parameter | Raw value | RPM | Where set |
|---|---|---|---|
| `n_commDeacvHi` | 32 | 2 rpm | `BLDC_Init()` |
| `n_commAcvLo` | 16 | 1 rpm | `BLDC_Init()` |
| `n_stdStillDet` | 0 | disabled | `BLDC_Init()` |
| `dz_cntTrnsDetHi` | 2002 | disabled (> counter max) | `BLDC_Init()` |

### OPENLOOP timing reference (16 kHz ISR)

| Parameter | Cycles | Time |
|---|---|---|
| `OPENLOOP_ALIGN_DURATION` | 3200 | 200 ms |
| `OPENLOOP_ACCEL_DURATION` | 16000 | 1000 ms |
| At `OPENLOOP_DELTA_THETA_MAX=36` | — | ≈100 rpm mechanical |
| `t_errQual` (debounce to qualify error) | 1280 | 80 ms |
| `t_errDequal` (debounce to clear error) | 9600 | 600 ms |
