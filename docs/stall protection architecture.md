# UGV Stall Protection Architecture – Investigation & Design Basis

## 1. Goal

Adapt the hoverboard motor controller firmware for a UGV use-case where temporary wheel blockage under torque is an expected operational condition rather than an immediate fatal fault.

The target behavior is:

* Preserve hard electrical protections that prevent hardware destruction.
* Prevent aggressive “instant motor disable” behavior during short-duration wheel stalls.
* Allow the UGV to continue operating under temporary traction loss or obstacle interaction.
* De-rate torque gracefully during prolonged stalls instead of immediately forcing complete motor shutdown.
* Keep safety boundaries deterministic and understandable.

The architecture should separate:

1. **Hard electrical protection**

   * Immediate current chopping
   * MOSFET protection
   * Shoot-through prevention
   * Thermal protection
   * Catastrophic sensor failure handling

2. **Operational stall management**

   * Wheel blocked
   * High torque + low/no speed
   * Temporary terrain resistance
   * Startup friction
   * Obstacle pushing
   * Low-speed maneuvering

---

# 2. Background

The firmware originates from a hoverboard use-case.

Hoverboards assume:

* human rider safety first
* rapid disengagement preferred
* wheel blockage likely means rider instability or crash

UGV requirements differ:

* wheels may legitimately stall temporarily
* pushing against obstacles may be intentional
* mission completion may be more important than drivetrain longevity
* short-term over-stress may be acceptable

Observed current behavior:

* high torque request at low or zero wheel speed can trigger a fault
* controller enters OPEN_MODE
* motor torque collapses immediately
* wheel freewheels
* UGV becomes immobilized

Observed practical effect:

* vehicle may become helpless after wheel obstruction
* drivetrain protection is overly aggressive for robotic usage

---

# 3. Current Intended Architecture Proposal

## 3.1 High-level control (UGV onboard computer)

The onboard Linux/Radxa controller should implement:

* smooth torque ramp-up
* no aggressive step inputs
* fast torque release when operator releases command

Proposed behavior:

* torque ramps from 0 → 1000 over approximately 1500 ms
* release drops immediately back to 0
* no command latching
* no sustained integrator buildup in upper layers

Reasoning:

* reduce drivetrain shock
* reduce false stall detection
* reduce current spikes
* keep motor controller operating inside expected dynamic envelope

---

## 3.2 Firmware-level hard protection

Hard protections should remain active:

* instantaneous current chopping
* PWM disable on overcurrent
* thermal shutdown
* invalid hall states
* hardware safety protections

These protections exist in `bldc.c` and should generally remain untouched.

Relevant observed logic:

```c
if(ABS(curL_DC) > curDC_max || enable == 0) {
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
}
```

This is considered:

* electrical safety protection
* hard real-time
* appropriate as-is

---

## 3.3 Firmware-level stall behavior adaptation

The aggressive operational stall handling should be softened.

Proposed behavior:

* ignore blocked-wheel condition initially for approximately 1000 ms
* after qualification timeout:

  * gradually reduce allowed torque
  * linearly ramp toward safer torque level
  * example sustained limit: ~350/1000
* avoid immediate OPEN_MODE transition where possible

Intent:

* preserve mobility
* reduce thermal stress
* allow temporary obstacle interaction
* avoid full drivetrain collapse

---
# 3.4 Proposed Stall Derating Timeline

The intended operational stall behavior is not merely:

* “wait longer before faulting”

but rather:

* temporarily tolerate high torque at low/no speed
* then gradually reduce stress if the stall persists

Proposed sequence:

## Phase 1 — Full-Torque Grace Period

Condition:

* wheel considered stalled or near-stalled
* high torque request active
* commutation not progressing normally

Behavior:

* allow full requested torque
* no torque derating yet
* no OPEN_MODE transition

Duration:

* approximately 1000 ms

Intent:

* allow obstacle climbing
* allow temporary wheel blockage
* avoid nuisance trips
* preserve mobility during transient resistance

---

## Phase 2 — Controlled Torque Derating

If stall condition persists beyond grace period:

Behavior:

* begin linear torque reduction
* gradually reduce allowed torque ceiling
* avoid abrupt torque collapse

Example:

* torque ceiling ramps from:

  * 1000
  * down to 350

over:

* approximately 1000 ms

Result:

* initial obstacle-push capability retained
* sustained drivetrain abuse reduced
* thermal stress lowered
* current demand reduced
* mobility partially preserved

The desired behavior is therefore:

```text
0 ms       : stall begins
0-1000 ms  : full torque allowed
1000-2000 ms : linear ramp-down from 1000 → 350
>2000 ms   : sustained torque limited to ~350
```

This should ideally recover automatically if:

* hall transitions resume
* speed recovers
* torque demand falls

---

# 3.5 Coordination Between Soft and Hard Protection Layers

A critical architectural constraint:

The hard protection layer must never trigger inside the intended operating envelope of the soft stall-management layer.

Otherwise:

* the soft derating logic becomes meaningless
* the system faults before the controlled ramp-down can complete

Example of BAD coordination:

```text
soft logic:
- allow 1000 torque for 1000 ms
- ramp toward 350 over next 1000 ms

hard logic:
- trigger fatal protection if torque >650 for 1200 ms
```

In this scenario:

* hard protection interrupts the soft ramp
* OPEN_MODE still occurs unexpectedly
* operational behavior remains unstable/unpredictable

Therefore:

The hard protection envelope must remain outside the full soft-management trajectory.

---

# 3.6 Candidate Implementation Locations

Two main architectural directions currently exist.

---

## Option A — Stall Management in `main.c`

Approach:

* keep BLDC controller mostly unchanged
* implement supervisory torque limiting externally
* manipulate requested torque before it reaches controller

Example:

* onboard logic requests 1000
* supervisory layer gradually reduces command to 350 during prolonged stall

Advantages:

* safer
* easier to maintain
* avoids invasive modification of autogenerated Simulink logic
* easier debugging
* lower risk of destabilizing FOC internals
* easier rollback/testing

Disadvantages:

* BLDC internal diagnostics may still trigger first
* requires hard fault thresholds to remain outside soft envelope
* limited visibility into internal controller state
* may duplicate logic already partially present in diagnostics

Important implication:

* internal hard diagnostics MUST be relaxed enough to allow the external derating strategy to complete.

---

## Option B — Stall Management Inside BLDC Diagnostics

Approach:

* modify qualification/dequalification logic directly
* integrate derating behavior inside firmware control architecture
* potentially alter diagnostic fault state machine itself

Advantages:

* direct access to:

  * hall state
  * electrical angle
  * commutation timing
  * torque request
  * internal controller states
* cleaner conceptual integration
* diagnostics can distinguish:

  * operational stall
  * catastrophic fault
* avoids duplicated external logic

Disadvantages:

* significantly riskier
* autogenerated Simulink code is difficult to maintain
* future regeneration may overwrite changes
* harder debugging
* greater risk of introducing unstable edge cases
* easier to accidentally destabilize FOC behavior

Additional concern:

* modifications inside fault-management paths can create subtle unsafe states that are difficult to observe during bench testing.

---

# 3.7 Current Preferred Direction

Current preferred architecture:

* retain hard electrical protections in `bldc.c`
* avoid modifying core FOC behavior
* avoid invasive edits to autogenerated state machines initially
* implement supervisory soft stall management externally where possible
* only relax/tune internal qualification timing enough to permit the external derating strategy to operate correctly

This approach minimizes:

* destabilization risk
* maintenance complexity
* unintended control-loop interactions

while still substantially improving operational robustness for UGV usage.

# 4. Architectural Principle

The system should distinguish between:

## A) Electrical faults (fatal)

Examples:

* overcurrent
* invalid hall states
* phase desynchronization
* thermal runaway
* hardware faults

Desired behavior:

* immediate shutdown or OPEN_MODE

---

## B) Operational stalls (non-fatal)

Examples:

* wheel blocked
* terrain resistance
* pushing obstacle
* low-speed maneuvering
* startup friction

Desired behavior:

* temporary tolerance
* torque derating
* graceful degradation
* recovery possible without full controller reset

---

# 5. Findings From Initial Reverse Engineering

## 5.1 OPEN_MODE transition discovered

Inside autogenerated `BLDC_controller.c`:

```c
rtb_RelationalOperator1_mv = (
    rtDW->Merge_p ||
    (!rtU->b_motEna) ||
    (rtU->z_ctrlModReq == 0)
);
```

This condition forces:

```c
rtDW->is_c1_BLDC_controller = IN_OPEN;
rtDW->z_ctrlMod = OPEN_MODE;
```

Observed consequence:

* torque collapses
* wheel becomes passive/freewheel
* UGV loses drive capability

---

## 5.2 Diagnostic fault path identified

Current understanding:

```text
diagnostic condition
    ↓
qualification logic
    ↓
z_errCode
    ↓
Merge_p
    ↓
forced OPEN_MODE
    ↓
loss of torque
```

---

## 5.3 Evidence of qualification timers

Relevant identifiers discovered:

* `t_errQual`
* `t_errDequal`
* `CTRL_COMM2`

Interpretation:

* `t_errQual`
  = error qualification time
* `t_errDequal`
  = error dequalification time
* `CTRL_COMM2`
  likely related to commutation-related fault detection

This strongly suggests:

* stall protection is already time-qualified
* fault assertion is likely NOT purely instantaneous
* fault recovery may already contain hysteresis/dequalification

---

# 6. Current Hypothesis

The problematic detector likely exists inside:

```text
F02_Diagnostics
Diagnostics_Enabled
CTRL_COMM2
```

Most likely trigger mechanisms:

* high torque + missing hall transitions
* commutation expected but rotor not advancing
* electrical angle divergence
* stalled rotor detection

Less likely:

* simple current threshold alone

---

# 7. Strong Recommendation

Do NOT:

* remove hard current chopping
* bypass electrical protections
* disable OPEN_MODE globally
* directly edit autogenerated Simulink state logic without understanding the fault path

Preferred strategy:

1. locate qualification logic
2. identify stall detector inputs
3. extend qualification timing
4. distinguish operational stall from fatal fault
5. introduce torque derating before full OPEN_MODE shutdown

---

# 8. Requested Codex Investigation

Codex should:

## A) Locate exact detector implementation

Search for:

* `CTRL_COMM2`
* `t_errQual`
* `t_errDequal`
* `Merge_p`
* `z_errCode`

Goal:

* identify exact stall detection condition
* determine whether it is:

  * timer-based
  * counter-based
  * hall-transition based
  * current-based
  * angle-divergence based

---

## B) Document current protection flow

Create a precise map:

```text
condition
→ qualification
→ fault code
→ Merge_p
→ OPEN_MODE
```

---

## C) Determine safest modification point

Codex should recommend:

* where to implement operational stall tolerance
* whether modifications belong in:

  * autogenerated logic
  * wrapper logic
  * bldc.c
  * main.c
  * external torque management

Preference:

* avoid invasive modification of autogenerated Simulink state machine where possible

---

## D) Propose implementation options

At minimum compare:

### Option 1

Increase qualification/dequalification timers only.

### Option 2

Introduce torque derating before fault assertion.

### Option 3

Differentiate fatal vs recoverable faults.

### Option 4

External supervisory stall management in `main.c`.

Each option should include:

* safety implications
* implementation complexity
* maintainability
* robustness
* risk of unstable behavior

---

# 9. Important Constraints

* UGV operates in harsh conditions.
* Temporary drivetrain abuse is acceptable.
* Mission continuity is prioritized higher than ideal drivetrain lifetime.
* Hard electrical destruction prevention remains mandatory.
* Behavior must remain deterministic.
* Avoid modifications that destabilize FOC control loops.

---

# 10. Additional Notes

The firmware already contains:

* open-loop startup experimentation
* gradual torque command filtering
* ISR-level current protection
* mode state machines
* diagnostic qualification logic

The requested architecture should integrate with these existing mechanisms instead of replacing them.
