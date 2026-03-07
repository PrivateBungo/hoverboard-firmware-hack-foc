#pragma once
/*
 * openloop_debug.h -- Debug interface for OPENLOOP internal state and final
 *                     applied PWM commands.
 *
 * OpenLoopSnapshot: safe read of bldc.c open-loop state variables into a
 * plain struct.  The getter disables interrupts briefly to avoid torn reads
 * from the 16 kHz DMA ISR.  Defined unconditionally so main.c can use a
 * zero-initialised fallback when OPENLOOP_ENABLE is not defined.
 *
 * AppliedPwm3: final phase commands written to the inverter (ul/vl/wl after
 * all overrides).  Defined here so the typedef is shared between bldc.c
 * (which writes the volatile globals) and main.c (which reads them).
 */

#include <stdint.h>

typedef struct {
    int16_t  phase;        /* 0=inactive, 1=align, 2=rotate */
    int16_t  theta;        /* synthetic electrical angle [0..23039] (electrical angle units, same as plook table) */
    int16_t  delta_theta;  /* angle increment per ISR cycle [units/ISR] (signed; negative = reverse) */
    int16_t  voltage;      /* voltage amplitude [0..OPENLOOP_VOLTAGE_MAX] */
} OpenLoopSnapshot;

/* Final applied PWM phase commands captured immediately before the timer CCR
 * writes in the ISR.  Written in the ISR, read in the main loop for CSV
 * logging. */
typedef struct { int16_t u, v, w; } AppliedPwm3;

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
