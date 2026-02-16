#include "persist_config.h"

#include "stm32f1xx_hal.h"

#define PERSIST_CONFIG_ABS32(a) (((a) < 0) ? (-(a)) : (a))

#define PERSIST_CONFIG_MAGIC          (0x4E454355UL)
#define PERSIST_CONFIG_VERSION        (1U)
#define PERSIST_CONFIG_MIN_SAVE_DELTA (2)

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  int16_t neutral_pwml;
  int16_t neutral_pwmr;
  uint16_t checksum;
} PersistNeutralConfig;

#define PERSIST_CONFIG_FLASH_ADDR \
  (FLASH_BASE + (((uint32_t)(*(uint16_t *)FLASHSIZE_BASE)) * 1024U) - FLASH_PAGE_SIZE)

static uint16_t PersistConfig_Checksum(const PersistNeutralConfig *cfg) {
  const uint8_t *raw;
  uint16_t checksum;
  uint16_t idx;

  if (cfg == 0) {
    return 0U;
  }

  raw = (const uint8_t *)cfg;
  checksum = 0U;

  for (idx = 0U; idx < (uint16_t)(sizeof(PersistNeutralConfig) - sizeof(cfg->checksum)); idx++) {
    checksum = (uint16_t)(checksum + raw[idx]);
  }

  return checksum;
}

static uint8_t PersistConfig_IsValid(const PersistNeutralConfig *cfg) {
  if (cfg == 0) {
    return 0U;
  }

  if (cfg->magic != PERSIST_CONFIG_MAGIC) {
    return 0U;
  }

  if (cfg->version != PERSIST_CONFIG_VERSION) {
    return 0U;
  }

  if (cfg->size != (uint16_t)sizeof(PersistNeutralConfig)) {
    return 0U;
  }

  return (uint8_t)(cfg->checksum == PersistConfig_Checksum(cfg));
}

uint8_t PersistConfig_LoadNeutral(int16_t *neutral_pwml, int16_t *neutral_pwmr) {
  const PersistNeutralConfig *cfg;

  if ((neutral_pwml == 0) || (neutral_pwmr == 0)) {
    return 0U;
  }

  cfg = (const PersistNeutralConfig *)PERSIST_CONFIG_FLASH_ADDR;
  if (PersistConfig_IsValid(cfg) == 0U) {
    return 0U;
  }

  *neutral_pwml = cfg->neutral_pwml;
  *neutral_pwmr = cfg->neutral_pwmr;
  return 1U;
}

uint8_t PersistConfig_SaveNeutral(int16_t neutral_pwml, int16_t neutral_pwmr) {
  PersistNeutralConfig cfg;
  int16_t currentLeft;
  int16_t currentRight;
  FLASH_EraseInitTypeDef erase;
  uint32_t pageError;
  uint32_t addr;
  uint16_t *raw;
  uint16_t idx;

  if (PersistConfig_LoadNeutral(&currentLeft, &currentRight) != 0U) {
    if ((PERSIST_CONFIG_ABS32((int32_t)neutral_pwml - (int32_t)currentLeft) < PERSIST_CONFIG_MIN_SAVE_DELTA) &&
        (PERSIST_CONFIG_ABS32((int32_t)neutral_pwmr - (int32_t)currentRight) < PERSIST_CONFIG_MIN_SAVE_DELTA)) {
      return 1U;
    }
  }

  cfg.magic = PERSIST_CONFIG_MAGIC;
  cfg.version = PERSIST_CONFIG_VERSION;
  cfg.size = (uint16_t)sizeof(PersistNeutralConfig);
  cfg.neutral_pwml = neutral_pwml;
  cfg.neutral_pwmr = neutral_pwmr;
  cfg.checksum = PersistConfig_Checksum(&cfg);

  HAL_FLASH_Unlock();

  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = PERSIST_CONFIG_FLASH_ADDR;
  erase.NbPages = 1U;

  if (HAL_FLASHEx_Erase(&erase, &pageError) != HAL_OK) {
    HAL_FLASH_Lock();
    return 0U;
  }

  raw = (uint16_t *)&cfg;
  addr = PERSIST_CONFIG_FLASH_ADDR;
  for (idx = 0U; idx < (uint16_t)(sizeof(PersistNeutralConfig) / sizeof(uint16_t)); idx++) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, raw[idx]) != HAL_OK) {
      HAL_FLASH_Lock();
      return 0U;
    }
    addr += sizeof(uint16_t);
  }

  HAL_FLASH_Lock();
  return 1U;
}
