#ifndef VELOCITY_SETPOINT_TUNING_H
#define VELOCITY_SETPOINT_TUNING_H

// Longitudinal torque command slew shaping (TRQ mode)
#define LONG_RAMP_UP_NUM         6         // [-] numerator for up-ramp scaling vs RATE (6/5 => +20% faster than current)
#define LONG_RAMP_UP_DEN         5         // [-] denominator for up-ramp scaling vs RATE
#define LONG_RAMP_DOWN_NUM       2         // [-] numerator for down-ramp scaling vs up-ramp (2x faster down than up)
#define LONG_RAMP_DOWN_DEN       1         // [-] denominator for down-ramp scaling vs up-ramp

// Velocity setpoint shaping in physical units (Step D)
// Reference wheel: 40 cm diameter (0.4 m)
#define SETPOINT_WHEEL_DIAMETER_MM        400   // [mm] used to normalize m/s, m/s^2, and m/s^3 style tuning to command-domain increments
#define SETPOINT_WHEEL_CIRCUMFERENCE_MM   1257  // [mm] pi*D for D=0.4 m (rounded)
#define SETPOINT_SPEED_MAX_RPM_FALLBACK   1000  // [rpm] fallback if runtime speed max is unavailable/invalid

// Acceleration/jerk tuning (in SI-like units using millimeters)
#define SETPOINT_ACCEL_UP_MMPS2          2100    // [mm/s^2] forward acceleration shaping limit (~2.10 m/s^2)
#define SETPOINT_ACCEL_DOWN_MMPS2        4200   // [mm/s^2] deceleration shaping limit (~4.20 m/s^2)
#define SETPOINT_JERK_UP_MMPS3           10500   // [mm/s^3] accel-rise jerk limit (~10.50 m/s^3)
#define SETPOINT_JERK_DOWN_MMPS3         21000   // [mm/s^3] decel-rise jerk limit (~21.00 m/s^3)

// Slip-gap policy refinement
#define SETPOINT_SLIP_GAP_ON          300  // [-] clamp enters when |v_set - v_actual| exceeds this threshold
#define SETPOINT_SLIP_GAP_OFF         220  // [-] clamp may release only when |v_set - v_actual| is below this threshold
#define SETPOINT_SLIP_RELEASE_LOOPS   80   // [loops] continuous below-off duration required to release clamp (~0.4s at 5ms loop)

// Step D soft-limit integration (outside generated hard-limit ownership)
#define SOFT_LIMIT_TORQUE_WHEN_SLIP   850  // [-] max |torque cmd| while slip-gap clamp is active

#endif // VELOCITY_SETPOINT_TUNING_H
