#ifndef MOTOR_CONTROLLER_GAINS_H
#define MOTOR_CONTROLLER_GAINS_H

/*
 * IMPORTANT CLARIFICATION:
 * - Generated BLDC controller PI/PID internals (high-frequency electrical/current-domain loops)
 *   remain owned by generated parameter sets in `Src/BLDC_controller_data.c`.
 * - This file is reserved for the planned outer velocity controller tuning gains
 *   (main-loop speed-domain PID/PI that tracks vehicle velocity setpoint).
 *
 * In other words: this file is NOT for replacing low-level FOC/current loops.
 * It is the dedicated home for the future speed-tracking controller introduced
 * in later rollout phases.
 */

#endif // MOTOR_CONTROLLER_GAINS_H
