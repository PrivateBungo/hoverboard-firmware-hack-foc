#ifndef COMMAND_FILTER_TUNING_H
#define COMMAND_FILTER_TUNING_H

/*
 * Boot-time neutral-offset calibration window:
 * - settle: wait for input interface to stabilize after power-up
 * - sample: average raw command while wheels are torque-inhibited
 */
#define COMMAND_FILTER_BOOT_SETTLE_MS        1000U
#define COMMAND_FILTER_BOOT_SAMPLE_MS        1000U

/* Longitudinal command smoothing (first-order LPF in Q15).
 * 32768 ~= 1.0 (no smoothing), smaller values = stronger smoothing. */
#define COMMAND_FILTER_LONGITUDINAL_LPF_ALPHA_Q15  4096

/* Offset update guardrails */
#define COMMAND_FILTER_LEARN_ZONE_ENTER      35
#define COMMAND_FILTER_LEARN_ZONE_EXIT       50
#define COMMAND_FILTER_LEARN_OPERATOR_ABORT  300
#define COMMAND_FILTER_LEARN_STABLE_DELTA    8
#define COMMAND_FILTER_LEARN_STABLE_COUNT_MIN 40U
#define COMMAND_FILTER_OFFSET_MAX            300

#endif // COMMAND_FILTER_TUNING_H
