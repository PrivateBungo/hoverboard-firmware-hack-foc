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

/**
 * HwGateSnapshot -- hardware gating / enable state captured in the ISR
 * immediately after the final CCR writes.
 *
 * These are the truth sources for whether PWM is actually being driven to each
 * motor.  Written in the 16 kHz DMA ISR; read by the main-loop CSV path.
 * Individual fields are ≤ 16 bits so reads are atomic on Cortex-M3; the
 * struct as a whole may be torn across ISR boundaries, which is acceptable for
 * debug logging.
 *
 * Fields are defined unconditionally so main.c can always declare the extern
 * and read the (zero-initialised) values even when OPENLOOP_ENABLE is not set.
 */
typedef struct {
    uint8_t  moe_l;    /* 1 = LEFT_TIM  (TIM8) BDTR.MOE set → outputs enabled */
    uint8_t  moe_r;    /* 1 = RIGHT_TIM (TIM1) BDTR.MOE set → outputs enabled */
    uint16_t ccer_l;   /* LEFT_TIM  CCER register (channel-enable mask, bits[11:0]) */
    uint16_t ccer_r;   /* RIGHT_TIM CCER register */
    uint8_t  enable;   /* firmware-level enable flag (main-loop safety gate) */
    uint8_t  enableFin;/* enableFin = enable && !errCodeL && !errCodeR (at time of CCR write) */
    uint16_t ccr_ul;   /* LEFT  motor CCR channel U (actual compare value written to timer) */
    uint16_t ccr_vl;   /* LEFT  motor CCR channel V */
    uint16_t ccr_wl;   /* LEFT  motor CCR channel W */
    uint16_t ccr_ur;   /* RIGHT motor CCR channel U */
    uint16_t ccr_vr;   /* RIGHT motor CCR channel V */
    uint16_t ccr_wr;   /* RIGHT motor CCR channel W */
} HwGateSnapshot;

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
