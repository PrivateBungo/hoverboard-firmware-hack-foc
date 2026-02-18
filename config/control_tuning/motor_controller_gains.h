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
 */

#define MOTOR_CTRL_VEL_KP_Q15    39321  // [-] Q15 gain (~1.20) from speed error [rpm] to torque command units
#define MOTOR_CTRL_VEL_KI_Q15     1638  // [-] Q15 integral gain (~0.05) per main-loop update
#define MOTOR_CTRL_INT_LIM       800    // [-] integrator clamp in torque-command units
#define MOTOR_CTRL_TORQUE_MAX   1000    // [-] outer-loop torque command saturation before soft limits

#endif // MOTOR_CONTROLLER_GAINS_H
