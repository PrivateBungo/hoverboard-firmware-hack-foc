#ifndef VELOCITY_SETPOINT_TUNING_H
#define VELOCITY_SETPOINT_TUNING_H

// Velocity setpoint shaping in physical units (Step D)
// Reference wheel: 40 cm diameter (0.4 m)
#define SETPOINT_WHEEL_DIAMETER_MM        400   // [mm] used to normalize m/s, m/s^2, and m/s^3 style tuning to command-domain increments
#define SETPOINT_WHEEL_CIRCUMFERENCE_MM   1257  // [mm] pi*D for D=0.4 m (rounded)
#define SETPOINT_SPEED_MAX_RPM_FALLBACK   1000  // [rpm] fallback if runtime speed max is unavailable/invalid

// Acceleration/jerk tuning (in SI-like units using millimeters)
#define SETPOINT_ACCEL_UP_MMPS2          700    // [mm/s^2] forward acceleration shaping limit (~0.70 m/s^2)
#define SETPOINT_ACCEL_DOWN_MMPS2        1400   // [mm/s^2] deceleration shaping limit (~1.40 m/s^2)
#define SETPOINT_JERK_UP_MMPS3           3500   // [mm/s^3] accel-rise jerk limit (~3.50 m/s^3)
#define SETPOINT_JERK_DOWN_MMPS3         7000   // [mm/s^3] decel-rise jerk limit (~7.00 m/s^3)

// Slip-gap policy refinement
#define SETPOINT_SLIP_GAP_ON          300  // [-] clamp enters when |v_set - v_actual| exceeds this threshold
#define SETPOINT_SLIP_GAP_OFF         220  // [-] clamp may release only when |v_set - v_actual| is below this threshold
#define SETPOINT_SLIP_RELEASE_LOOPS   80   // [loops] continuous below-off duration required to release clamp (~0.4s at 5ms loop)

// Soft-limit integration (outside generated hard-limit ownership)
#define SOFT_LIMIT_TORQUE_WHEN_SLIP   450  // [-] max |torque cmd| while slip-gap clamp is active

#endif // VELOCITY_SETPOINT_TUNING_H
