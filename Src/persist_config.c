#include "persist_config.h"

#include <string.h>

#include "stm32f1xx_hal.h"

#define PERSIST_CONFIG_MAGIC             (0x4E554F46UL) /* 'FOUN' */
#define PERSIST_CONFIG_VERSION           (1U)
#define PERSIST_FLASH_BASE_ADDR          (0x08000000UL)
#define PERSIST_FLASH_PAGE_SIZE_BYTES    (0x400UL)
#define PERSIST_SAVE_MIN_DELTA           (2)

typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  int16_t neutral_pwml;
  int16_t neutral_pwmr;
  uint16_t reserved;
  uint32_t checksum;
} PersistNeutralConfig;

static uint32_t PersistConfig_CalcChecksum(const PersistNeutralConfig *cfg) {
  uint32_t checksum = 0U;
  checksum ^= cfg->magic;
  checksum ^= (uint32_t)cfg->version;
  checksum ^= (uint32_t)cfg->size;
  checksum ^= (uint16_t)cfg->neutral_pwml;
  checksum ^= ((uint32_t)(uint16_t)cfg->neutral_pwmr << 16);
  checksum ^= (uint32_t)cfg->reserved;
  checksum ^= 0xA5A55A5AUL;
  return checksum;
}

static uint32_t PersistConfig_GetPageAddress(void) {
  uint32_t flashSizeKb = (*(uint16_t *)FLASHSIZE_BASE);
  uint32_t flashSizeBytes = flashSizeKb * 1024UL;
  uint32_t flashEndAddress = PERSIST_FLASH_BASE_ADDR + flashSizeBytes;
  return flashEndAddress - PERSIST_FLASH_PAGE_SIZE_BYTES;
}

uint8_t PersistConfig_LoadNeutral(int16_t *neutralPwml, int16_t *neutralPwmr) {
  PersistNeutralConfig cfg;
  const PersistNeutralConfig *cfgFlash = (const PersistNeutralConfig *)PersistConfig_GetPageAddress();

  if ((neutralPwml == 0) || (neutralPwmr == 0)) {
    return 0U;
  }

  memcpy(&cfg, cfgFlash, sizeof(cfg));

  if ((cfg.magic != PERSIST_CONFIG_MAGIC) ||
      (cfg.version != PERSIST_CONFIG_VERSION) ||
      (cfg.size != sizeof(PersistNeutralConfig)) ||
      (cfg.checksum != PersistConfig_CalcChecksum(&cfg))) {
    return 0U;
  }

  *neutralPwml = cfg.neutral_pwml;
  *neutralPwmr = cfg.neutral_pwmr;
  return 1U;
}

uint8_t PersistConfig_SaveNeutral(int16_t neutralPwml, int16_t neutralPwmr) {
  FLASH_EraseInitTypeDef erase;
  PersistNeutralConfig cfg;
  uint16_t i;
  uint16_t halfword;
  uint32_t pageAddress;
  uint32_t pageError = 0U;

  int16_t existingL = 0;
  int16_t existingR = 0;
  uint8_t existingValid = PersistConfig_LoadNeutral(&existingL, &existingR);

  if (existingValid != 0U) {
    int16_t deltaL = (existingL >= neutralPwml) ? (existingL - neutralPwml) : (neutralPwml - existingL);
    int16_t deltaR = (existingR >= neutralPwmr) ? (existingR - neutralPwmr) : (neutralPwmr - existingR);
    if ((deltaL < PERSIST_SAVE_MIN_DELTA) &&
        (deltaR < PERSIST_SAVE_MIN_DELTA)) {
      return 1U;
    }
  }

  cfg.magic = PERSIST_CONFIG_MAGIC;
  cfg.version = PERSIST_CONFIG_VERSION;
  cfg.size = sizeof(PersistNeutralConfig);
  cfg.neutral_pwml = neutralPwml;
  cfg.neutral_pwmr = neutralPwmr;
  cfg.reserved = 0U;
  cfg.checksum = PersistConfig_CalcChecksum(&cfg);

  pageAddress = PersistConfig_GetPageAddress();

  if (HAL_FLASH_Unlock() != HAL_OK) {
    return 0U;
  }

  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = pageAddress;
  erase.NbPages = 1U;

  if (HAL_FLASHEx_Erase(&erase, &pageError) != HAL_OK) {
    HAL_FLASH_Lock();
    return 0U;
  }

  for (i = 0U; i < (sizeof(cfg) / 2U); i++) {
    memcpy(&halfword, ((const uint8_t *)&cfg) + (i * 2U), sizeof(halfword));

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, pageAddress + (i * 2U), halfword) != HAL_OK) {
      HAL_FLASH_Lock();
      return 0U;
    }
  }

  HAL_FLASH_Lock();
  return 1U;
}
