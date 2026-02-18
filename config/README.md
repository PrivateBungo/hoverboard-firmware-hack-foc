# config

Layered configuration introduced by iteration 5 of the repository architecture cleanup plan.

- `feature/feature_flags.h`: build-time feature/variant selection macros.
- `control/control_defaults.h`: control-loop timing, PWM, and ADC conversion constants.
- `control_tuning/command_filter_tuning.h`: command-filter neutral-offset calibration windows/guards for both steering and longitudinal axes.
- `control_tuning/intent_and_input_tuning.h`: user-intent hysteresis, measured-speed near-zero thresholds, and ZERO_LATCH hold timing.
- `control_tuning/velocity_setpoint_tuning.h`: longitudinal speed-intent mapping and setpoint-shaper tuning (`LONG_*`, `SETPOINT_*`).
- `control_tuning/motor_controller_gains.h`: dedicated home for the future outer velocity-controller (speed-domain PID/PI) gains; distinct from generated high-frequency current-loop gains.
- `board/board_default.h`: default board-level hardware mapping selection.
- `user/user_params.h`: user-tunable defaults (timeouts, filtering, and command shaping coefficients).

`Inc/config.h` remains the compatibility facade include used by existing source files and now composes these layered headers before applying the rest of the legacy configuration blocks.
