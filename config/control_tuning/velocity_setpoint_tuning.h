#ifndef VELOCITY_SETPOINT_TUNING_H
#define VELOCITY_SETPOINT_TUNING_H

// Longitudinal speed-intent to torque-request mapping (TRQ mode)
#define LONG_SPEED_KP_Q15        32768     // [-] fixdt(1,16,15) proportional gain from speed error to torque request
#define LONG_RAMP_UP_NUM         6         // [-] numerator for up-ramp scaling vs RATE (6/5 => +20% faster than current)
#define LONG_RAMP_UP_DEN         5         // [-] denominator for up-ramp scaling vs RATE
#define LONG_RAMP_DOWN_NUM       2         // [-] numerator for down-ramp scaling vs up-ramp (2x faster down than up)
#define LONG_RAMP_DOWN_DEN       1         // [-] denominator for down-ramp scaling vs up-ramp

// Velocity setpoint shaping (Step C: asymmetric trajectory shaping)
#define SETPOINT_RATE_UP         5         // [-] per-loop max increase for velocity setpoint (softer throttle build-up)
#define SETPOINT_RATE_DOWN       10        // [-] per-loop max decrease for velocity setpoint (faster release/braking response)
#define SETPOINT_SLIP_GAP_MAX    300       // [-] max |v_set - v_actual| gap before clamping to reduce wheel-slip transients

#endif // VELOCITY_SETPOINT_TUNING_H
