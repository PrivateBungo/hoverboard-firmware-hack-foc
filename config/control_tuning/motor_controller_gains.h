#ifndef MOTOR_CONTROLLER_GAINS_H
#define MOTOR_CONTROLLER_GAINS_H

/*
 * IMPORTANT CLARIFICATION:
 * - Generated BLDC controller PI/PID internals (high-frequency electrical/current-domain loops)
 *   remain owned by generated parameter sets in `Src/BLDC_controller_data.c`.
 * - The constants below tune ONLY the outer velocity PI loop running in the
 *   main loop speed domain. The outer loop maps velocity setpoint tracking
 *   error to torque command request in command units [-1000..1000].
 */

// Outer velocity PI gains in Q15 fixed-point.
#define LONG_SPEED_KP_Q15                 40960   // [-] 1.250 P gain from normalized speed error to torque request
#define LONG_SPEED_KI_Q15                 8192    // [-] 0.250 I gain per main-loop update (5 ms nominal)

// Integrator and output limits in command units.
#define LONG_SPEED_I_TERM_MAX            1000     // [-] max positive integrator contribution
#define LONG_SPEED_I_TERM_MIN           -1000     // [-] min negative integrator contribution
#define LONG_SPEED_OUTER_TORQUE_CMD_MAX  1000     // [-] outer-loop hard saturation (pre soft-limit path)
#define LONG_SPEED_OUTER_TORQUE_CMD_MIN -1000     // [-] outer-loop hard saturation (pre soft-limit path)

#endif // MOTOR_CONTROLLER_GAINS_H
