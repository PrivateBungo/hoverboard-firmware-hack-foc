#ifndef VELOCITY_SETPOINT_TUNING_H
#define VELOCITY_SETPOINT_TUNING_H

// Longitudinal speed-intent to torque-request mapping (TRQ mode)
#define LONG_SPEED_KP_Q15        32768     // [-] fixdt(1,16,15) proportional gain from speed error to torque request
#define LONG_RAMP_UP_NUM         6         // [-] numerator for up-ramp scaling vs RATE (6/5 => +20% faster than current)
#define LONG_RAMP_UP_DEN         5         // [-] denominator for up-ramp scaling vs RATE
#define LONG_RAMP_DOWN_NUM       2         // [-] numerator for down-ramp scaling vs up-ramp (2x faster down than up)
#define LONG_RAMP_DOWN_DEN       1         // [-] denominator for down-ramp scaling vs up-ramp

// Velocity setpoint shaping (Step D: slip-gap policy refinement + release hysteresis)
#define SETPOINT_RATE_UP              5    // [-] per-loop max increase for velocity setpoint (softer throttle build-up)
#define SETPOINT_RATE_DOWN            10   // [-] per-loop max decrease for velocity setpoint (faster release/braking response)
#define SETPOINT_SLIP_GAP_ON          300  // [-] clamp enters when |v_set - v_actual| exceeds this threshold
#define SETPOINT_SLIP_GAP_OFF         220  // [-] clamp may release only when |v_set - v_actual| is below this threshold
#define SETPOINT_SLIP_RELEASE_LOOPS   80   // [loops] continuous below-off duration required to release clamp (~0.4s at 5ms loop)

// Step D soft-limit integration (outside generated hard-limit ownership)
#define SOFT_LIMIT_TORQUE_WHEN_SLIP   450  // [-] max |torque cmd| while slip-gap clamp is active

#endif // VELOCITY_SETPOINT_TUNING_H
