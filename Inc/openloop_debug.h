#pragma once
/*
 * openloop_debug.h -- Snapshot interface for OPENLOOP internal state.
 *
 * Provides a safe read of the bldc.c open-loop state variables into a plain
 * struct so that main.c can include them in the CSV debug stream without
 * dereferencing the static variables directly.
 *
 * The getter disables interrupts briefly to avoid torn reads of the fields
 * that are updated inside the 16 kHz DMA ISR.
 *
 * OpenLoopSnapshot is always defined so that main.c can use a zero-initialised
 * fallback struct when OPENLOOP_ENABLE is not defined.  The getter function is
 * only compiled when OPENLOOP_ENABLE is defined.
 */

#include <stdint.h>

typedef struct {
    int16_t  phase;        /* 0=inactive, 1=align, 2=rotate */
    int16_t  theta;        /* synthetic electrical angle [0..23039] (electrical angle units, same as plook table) */
    int16_t  delta_theta;  /* angle increment per ISR cycle [units/ISR] (signed; negative = reverse) */
    int16_t  voltage;      /* voltage amplitude [0..OPENLOOP_VOLTAGE_MAX] */
} OpenLoopSnapshot;

#ifdef OPENLOOP_ENABLE

/**
 * openloop_get_snapshot() -- atomically copy open-loop state for both motors.
 *
 * Disables interrupts for the duration of the copy to prevent torn reads from
 * the 16 kHz DMA ISR that updates olStateL/olStateR.  Both output pointers
 * must be non-NULL.
 */
void openloop_get_snapshot(OpenLoopSnapshot *left, OpenLoopSnapshot *right);

#endif /* OPENLOOP_ENABLE */
