#ifndef PERSIST_CONFIG_H
#define PERSIST_CONFIG_H

#include <stdint.h>

uint8_t PersistConfig_LoadNeutral(int16_t *neutralPwml, int16_t *neutralPwmr);
uint8_t PersistConfig_SaveNeutral(int16_t neutralPwml, int16_t neutralPwmr);

#endif /* PERSIST_CONFIG_H */
