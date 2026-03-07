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
 * This header and the corresponding implementation in Src/bldc.c are compiled
 * only when OPENLOOP_ENABLE is defined.
 */

#ifdef OPENLOOP_ENABLE

#include <stdint.h>

typedef struct {
    int16_t  phase;        /* 0=inactive, 1=align, 2=rotate */
    int16_t  theta;        /* synthetic electrical angle [0..23039] */
    int16_t  delta_theta;  /* angle increment per ISR cycle (signed) */
    int16_t  voltage;      /* voltage amplitude */
} OpenLoopSnapshot;

/**
 * openloop_get_snapshot() -- atomically copy open-loop state for both motors.
 *
 * Disables interrupts for the duration of the copy to prevent torn reads from
 * the 16 kHz ISR. Both output pointers must be non-NULL.
 */
void openloop_get_snapshot(OpenLoopSnapshot *left, OpenLoopSnapshot *right);

#endif /* OPENLOOP_ENABLE */
