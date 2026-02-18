#ifndef INTENT_AND_INPUT_TUNING_H
#define INTENT_AND_INPUT_TUNING_H

/* User-intent command hysteresis (input policy layer). */
#define USER_INTENT_HYST_ON                      50
#define USER_INTENT_HYST_OFF                     35

/* Intent-state-machine deadband and near-zero logic (measured-speed based). */
#define INTENT_STATE_MACHINE_CMD_DEADBAND        35
#define INTENT_STATE_MACHINE_NEAR_ZERO_ENTER     35
#define INTENT_STATE_MACHINE_NEAR_ZERO_EXIT      50

/* ZERO_LATCH hold timing (dynamic tuning knob in milliseconds). */
#define INTENT_STATE_MACHINE_ZERO_LATCH_HOLD_MS  500U

/* Main loop is nominally ~5 ms; keep explicit overrideable tick for intent timing. */
#define INTENT_STATE_MACHINE_TICK_MS             ((uint16_t)(DELAY_IN_MAIN_LOOP + 1U))

#endif // INTENT_AND_INPUT_TUNING_H
