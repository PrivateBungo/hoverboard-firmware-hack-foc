# config

Layered configuration introduced by iteration 5 of the repository architecture cleanup plan.

- `feature/feature_flags.h`: build-time feature/variant selection macros.
- `control/control_defaults.h`: control-loop timing, PWM, and ADC conversion constants.
- `board/board_default.h`: default board-level hardware mapping selection.
- `user/user_params.h`: user-tunable defaults (timeouts, filtering, and command shaping coefficients).

`Inc/config.h` remains the compatibility facade include used by existing source files and now composes these layered headers before applying the rest of the legacy configuration blocks.
