#include "boot_neutral_supervisor.h"

#include <stddef.h>
#include <stdio.h>

#include "config.h"
#include "persist_config.h"
#include "stm32f1xx_hal.h"
#include "util.h"

#define BOOT_NEUTRAL_OBSERVE_MS  (2000U)
#define BOOT_NEUTRAL_STABLE_BAND (30)
#define BOOT_NEUTRAL_ABS(a)      (((a) < 0) ? (-(a)) : (a))

#define PERSIST_CONFIG_FLASH_ADDR_DBG \
  (FLASH_BASE + (((uint32_t)(*(uint16_t *)FLASHSIZE_BASE)) * 1024U) - FLASH_PAGE_SIZE)
#define PERSIST_CONFIG_MAGIC_DBG (0x4E454355UL)

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  int16_t neutral_pwml;
  int16_t neutral_pwmr;
  uint16_t checksum;
} PersistNeutralConfigRaw;

static uint16_t BootNeutralSupervisor_DebugPersistChecksum(const PersistNeutralConfigRaw *cfg) {
  const uint8_t *raw;
  uint16_t checksum;
  uint16_t idx;

  if (cfg == 0) {
    return 0U;
  }

  raw = (const uint8_t *)cfg;
  checksum = 0U;
  for (idx = 0U; idx < (uint16_t)(offsetof(PersistNeutralConfigRaw, checksum)); idx++) {
    checksum = (uint16_t)(checksum + raw[idx]);
  }

  return checksum;
}

static void BootNeutralSupervisor_DebugPrintPersistNeutral(void) {
  const PersistNeutralConfigRaw *cfg;
  int16_t neutralL;
  int16_t neutralR;
  uint8_t valid;
  uint16_t checksumCalc;

  cfg = (const PersistNeutralConfigRaw *)PERSIST_CONFIG_FLASH_ADDR_DBG;
  checksumCalc = BootNeutralSupervisor_DebugPersistChecksum(cfg);
  valid = PersistConfig_LoadNeutral(&neutralL, &neutralR);

  if (valid != 0U) {
    printf("PersistNeutral: valid=%u magic=0x%08lX ver=%u size=%u chk=0x%04X chk_calc=0x%04X nL=%i nR=%i\r\n",
           (unsigned)valid,
           (unsigned long)cfg->magic,
           (unsigned)cfg->version,
           (unsigned)cfg->size,
           (unsigned)cfg->checksum,
           (unsigned)checksumCalc,
           (int)neutralL,
           (int)neutralR);
  } else {
    printf("PersistNeutral: valid=0 (no stored config) magic=0x%08lX ver=%u size=%u chk=0x%04X chk_calc=0x%04X nLraw=%i nRraw=%i expMagic=0x%08lX\r\n",
           (unsigned long)cfg->magic,
           (unsigned)cfg->version,
           (unsigned)cfg->size,
           (unsigned)cfg->checksum,
           (unsigned)checksumCalc,
           (int)cfg->neutral_pwml,
           (int)cfg->neutral_pwmr,
           (unsigned long)PERSIST_CONFIG_MAGIC_DBG);
  }
}

static void BootNeutralSupervisor_DebugPrintApplyDecision(const BootNeutralSupervisorState *state,
                                                          uint8_t learned,
                                                          uint8_t saveOk,
                                                          uint8_t loadValid,
                                                          int16_t neutralL,
                                                          int16_t neutralR) {
  if (learned != 0U) {
    printf("BOOTAPPLY rcAll=%u stabAll=%u n=%u learn nL=%i nR=%i save=%s beep=1\r\n",
           (unsigned)state->rc_present_all_window,
           (unsigned)state->stable_neutral_all_window,
           (unsigned)state->sampleCount,
           (int)neutralL,
           (int)neutralR,
           (saveOk != 0U) ? "OK" : "FAIL");
  } else {
    printf("BOOTAPPLY rcAll=%u stabAll=%u n=%u load valid=%u nL=%i nR=%i beep=0\r\n",
           (unsigned)state->rc_present_all_window,
           (unsigned)state->stable_neutral_all_window,
           (unsigned)state->sampleCount,
           (unsigned)loadValid,
           (int)neutralL,
           (int)neutralR);
  }
}

void BootNeutralSupervisor_Init(BootNeutralSupervisorState *state, uint32_t nowMs) {
  if (state == 0) {
    return;
  }

  state->state = BOOT_NEUTRAL_STATE_OBSERVE;
  state->boot_t0 = nowMs;
  state->rc_present_all_window = 1U;
  state->stable_neutral_all_window = 1U;
  state->sumLeft = 0;
  state->sumRight = 0;
  state->sampleCount = 0U;
  state->neutralLeft = 0;
  state->neutralRight = 0;
  state->neutral_active = 0U;
  state->bootDbgLastMs = 0U;

  BootNeutralSupervisor_DebugPrintPersistNeutral();
}

uint8_t BootNeutralSupervisor_IsRcInputSignalPresent(uint8_t inIdx,
                                                     uint8_t timeoutFlgGen,
                                                     uint8_t timeoutFlgSerial) {
  uint8_t present = 0U;

  #if defined(CONTROL_PWM_LEFT)
    if (inIdx == CONTROL_PWM_LEFT) {
      present = (uint8_t)(timeoutFlgGen == 0U);
    }
  #endif

  #if defined(CONTROL_PWM_RIGHT)
    if (inIdx == CONTROL_PWM_RIGHT) {
      present = (uint8_t)(timeoutFlgGen == 0U);
    }
  #endif

  #if defined(CONTROL_PPM_LEFT)
    if (inIdx == CONTROL_PPM_LEFT) {
      present = (uint8_t)(timeoutFlgGen == 0U);
    }
  #endif

  #if defined(CONTROL_PPM_RIGHT)
    if (inIdx == CONTROL_PPM_RIGHT) {
      present = (uint8_t)(timeoutFlgGen == 0U);
    }
  #endif

  #if defined(CONTROL_SERIAL_USART2)
    if (inIdx == CONTROL_SERIAL_USART2) {
      present = (uint8_t)(timeoutFlgSerial == 0U);
    }
  #endif

  #if defined(CONTROL_SERIAL_USART3)
    if (inIdx == CONTROL_SERIAL_USART3) {
      present = (uint8_t)(timeoutFlgSerial == 0U);
    }
  #endif

  return present;
}

void BootNeutralSupervisor_Process(BootNeutralSupervisorState *state,
                                   uint32_t nowMs,
                                   uint8_t rcPresent,
                                   uint8_t timeoutFlgGen,
                                   uint8_t timeoutFlgSerial,
                                   uint8_t ctrlModReq,
                                   uint8_t enable,
                                   int16_t cmdL_filt,
                                   int16_t cmdR_filt,
                                   int16_t *cmdL_adj,
                                   int16_t *cmdR_adj,
                                   uint8_t *forcePwmZero) {
  uint8_t stableNow;

  if ((state == 0) || (cmdL_adj == 0) || (cmdR_adj == 0) || (forcePwmZero == 0)) {
    return;
  }

  stableNow = (uint8_t)((BOOT_NEUTRAL_ABS(cmdL_filt) <= BOOT_NEUTRAL_STABLE_BAND) &&
                        (BOOT_NEUTRAL_ABS(cmdR_filt) <= BOOT_NEUTRAL_STABLE_BAND));

  if (state->state == BOOT_NEUTRAL_STATE_OBSERVE) {
    if (rcPresent == 0U) {
      state->rc_present_all_window = 0U;
    }

    if (stableNow == 0U) {
      state->stable_neutral_all_window = 0U;
    }

    if (rcPresent != 0U) {
      state->sumLeft += cmdL_filt;
      state->sumRight += cmdR_filt;
      if (state->sampleCount < 65535U) {
        state->sampleCount++;
      }
    }

    if ((nowMs - state->boot_t0) >= BOOT_NEUTRAL_OBSERVE_MS) {
      state->state = BOOT_NEUTRAL_STATE_APPLY;
    }
  }

  if (state->state == BOOT_NEUTRAL_STATE_APPLY) {
    uint8_t learned = 0U;
    uint8_t saveOk = 0U;
    uint8_t loadValid = 0U;

    if ((state->rc_present_all_window != 0U) &&
        (state->stable_neutral_all_window != 0U) &&
        (state->sampleCount > 0U)) {
      state->neutralLeft = (int16_t)(state->sumLeft / (int32_t)state->sampleCount);
      state->neutralRight = (int16_t)(state->sumRight / (int32_t)state->sampleCount);
      learned = 1U;
      saveOk = PersistConfig_SaveNeutral(state->neutralLeft, state->neutralRight);
      if (saveOk != 0U) {
        beepShort(7);
        beepShort(7);
      }
    } else {
      int16_t savedLeft = 0;
      int16_t savedRight = 0;
      loadValid = PersistConfig_LoadNeutral(&savedLeft, &savedRight);
      if (loadValid != 0U) {
        state->neutralLeft = savedLeft;
        state->neutralRight = savedRight;
      }
    }

    BootNeutralSupervisor_DebugPrintApplyDecision(state,
                                                  learned,
                                                  saveOk,
                                                  loadValid,
                                                  state->neutralLeft,
                                                  state->neutralRight);

    state->neutral_active = 1U;
    state->state = BOOT_NEUTRAL_STATE_RUN;
  }

  *cmdL_adj = cmdL_filt;
  *cmdR_adj = cmdR_filt;
  if (state->neutral_active != 0U) {
    *cmdL_adj = (int16_t)(*cmdL_adj - state->neutralLeft);
    *cmdR_adj = (int16_t)(*cmdR_adj - state->neutralRight);
  }

  *forcePwmZero = (uint8_t)(state->state == BOOT_NEUTRAL_STATE_OBSERVE);

  if (((nowMs - state->boot_t0) <= 3000U) && ((nowMs - state->bootDbgLastMs) >= 200U)) {
    state->bootDbgLastMs = nowMs;
    printf("BOOTDBG t=%lu el=%lu st=%u nAct=%u rc=%u toGEN=%u toSER=%u stab=%u band=%u cmdF=(%i,%i) cmdA=(%i,%i) pwm=(%i,%i) ctrl=%u ena=%u\r\n",
           (unsigned long)nowMs,
           (unsigned long)(nowMs - state->boot_t0),
           (unsigned)state->state,
           (unsigned)state->neutral_active,
           (unsigned)rcPresent,
           (unsigned)timeoutFlgGen,
           (unsigned)timeoutFlgSerial,
           (unsigned)stableNow,
           (unsigned)BOOT_NEUTRAL_STABLE_BAND,
           (int)cmdL_filt,
           (int)cmdR_filt,
           (int)*cmdL_adj,
           (int)*cmdR_adj,
           *forcePwmZero != 0U ? 0 : (int)*cmdL_adj,
           *forcePwmZero != 0U ? 0 : (int)*cmdR_adj,
           (unsigned)ctrlModReq,
           (unsigned)enable);
  }
}
