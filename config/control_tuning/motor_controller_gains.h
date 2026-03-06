#ifndef MOTOR_CONTROLLER_GAINS_H
#define MOTOR_CONTROLLER_GAINS_H

/*
 * Outer velocity controller (Step E)
 * ----------------------------------
 * This controller runs in the main loop and maps velocity setpoint tracking
 * error (rpm-domain) into a normalized wheel torque command [-1000..1000].
 *
 * NOTE:
 * - Generated BLDC PI/PID internals (high-frequency electrical/current-domain loops)
 *   remain owned by `Src/BLDC_controller_data.c`.
 * - Gains below tune only this outer speed-domain PI controller.
 *
 * TUNING GUIDE
 * ------------
 * All gains are Q15 fixed-point: float_value = Q15_value / 32768.
 *
 * KP (proportional gain):
 *   Controls immediate torque response to speed error.
 *   Too high → oscillation / overshoot when load is suddenly removed.
 *   Too low  → sluggish speed tracking; setpoint not reached quickly.
 *   Start with 0.40 (13107) and increase in steps of 0.10 until response
 *   is acceptable without oscillation.
 *
 * KI (integral gain per main-loop tick at 200 Hz / 5 ms):
 *   Eliminates steady-state speed error.
 *   Too high → integrator winds up quickly → large overshoot when
 *              external load (e.g. hand on wheel) is suddenly removed.
 *   Too low  → slow steady-state correction, speed droops under load.
 *   Effective rate = KI_float / loop_period_s.  At 200 Hz, KI=0.02 gives
 *   4 torque-units/sec per 1 rpm of steady error.
 *
 * INT_LIM (integrator clamp, torque-command units):
 *   Limits how much the integral can accumulate.
 *   Set to ≤ TORQUE_MAX to prevent the integrator alone from saturating the
 *   output when the load is removed.  400 gives ample authority while
 *   reducing post-disturbance overshoot.
 *
 * TORQUE_MAX (output saturation):
 *   Hard ceiling on the torque command before soft limits.
 *   1000 = full range.  Reduce to limit peak current during tuning.
 */

#define MOTOR_CTRL_VEL_KP_Q15    13107  // [-] Q15 gain (~0.40) from speed error [rpm] to torque command units
#define MOTOR_CTRL_VEL_KI_Q15      655  // [-] Q15 integral gain (~0.02) per main-loop update
#define MOTOR_CTRL_INT_LIM         400  // [-] integrator clamp in torque-command units
#define MOTOR_CTRL_TORQUE_MAX     1000  // [-] outer-loop torque command saturation before soft limits

#endif // MOTOR_CONTROLLER_GAINS_H
