#ifndef PERSIST_CONFIG_H
#define PERSIST_CONFIG_H

#include <stdint.h>

uint8_t PersistConfig_LoadNeutral(int16_t *neutral_pwml, int16_t *neutral_pwmr);
uint8_t PersistConfig_SaveNeutral(int16_t neutral_pwml, int16_t neutral_pwmr);

#endif /* PERSIST_CONFIG_H */
