/*
* This file implements FOC motor control.
* This control method offers superior performanace
* compared to previous cummutation method. The new method features:
* ► reduced noise and vibrations
* ► smooth torque output
* ► improved motor efficiency -> lower energy consumption
*
* Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "util.h"
#include "foc_adapter.h"

// Matlab includes and defines - from auto-code generation
// ###############################################################################
#include "BLDC_controller.h"           /* Model's header file */
#include "rtwtypes.h"

// ###############################################################################

static int16_t pwm_margin;              /* This margin allows to have a window in the PWM signal for proper FOC Phase currents measurement */

extern uint8_t ctrlModReq;
static int16_t curDC_max = (I_DC_MAX * A2BIT_CONV);
int16_t curL_phaA = 0, curL_phaB = 0, curL_DC = 0;
int16_t curR_phaB = 0, curR_phaC = 0, curR_DC = 0;

volatile int pwml = 0;
volatile int pwmr = 0;

extern volatile adc_buf_t adc_buffer;

uint8_t buzzerFreq          = 0;
uint8_t buzzerPattern       = 0;
uint8_t buzzerCount         = 0;
volatile uint32_t buzzerTimer = 0;
static uint8_t  buzzerPrev  = 0;
static uint8_t  buzzerIdx   = 0;

uint8_t g_errCodeLeftEffective  = 0;
uint8_t g_errCodeRightEffective = 0;

uint8_t        enable       = 0;        // initially motors are disabled for SAFETY
static uint8_t enableFin    = 0;

static const uint16_t pwm_res  = 64000000 / 2 / PWM_FREQ; // = 2000

static uint16_t offsetcount = 0;
static int16_t offsetrlA    = 2000;
static int16_t offsetrlB    = 2000;
static int16_t offsetrrB    = 2000;
static int16_t offsetrrC    = 2000;
static int16_t offsetdcl    = 2000;
static int16_t offsetdcr    = 2000;

int16_t        batVoltage       = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;
static int32_t batVoltageFixdt  = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE << 16;  // Fixed-point filter output initialized at 400 V*100/cell = 4 V/cell converted to fixed-point

// =================================
// DMA interrupt frequency =~ 16 kHz
// =================================
#define STALL_ERR_BIT              (0x04U)
#define STALL_RECOVERY_DELAY_MS    (10000U)
#define STALL_NEUTRAL_PWM_DEADBAND (300)

void DMA1_Channel1_IRQHandler(void) {

  DMA1->IFCR = DMA_IFCR_CTCIF1;
  // HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
  // HAL_GPIO_TogglePin(LED_PORT, LED_PIN);

  if(offsetcount < 2000) {  // calibrate ADC offsets
    offsetcount++;
    offsetrlA = (adc_buffer.rlA + offsetrlA) / 2;
    offsetrlB = (adc_buffer.rlB + offsetrlB) / 2;
    offsetrrB = (adc_buffer.rrB + offsetrrB) / 2;
    offsetrrC = (adc_buffer.rrC + offsetrrC) / 2;
    offsetdcl = (adc_buffer.dcl + offsetdcl) / 2;
    offsetdcr = (adc_buffer.dcr + offsetdcr) / 2;
    return;
  }

  if (buzzerTimer % 1000 == 0) {  // Filter battery voltage at a slower sampling rate
    filtLowPass32(adc_buffer.batt1, BAT_FILT_COEF, &batVoltageFixdt);
    batVoltage = (int16_t)(batVoltageFixdt >> 16);  // convert fixed-point to integer
  }

  // Get Left motor currents
  curL_phaA = (int16_t)(offsetrlA - adc_buffer.rlA);
  curL_phaB = (int16_t)(offsetrlB - adc_buffer.rlB);
  curL_DC   = (int16_t)(offsetdcl - adc_buffer.dcl);
  
  // Get Right motor currents
  curR_phaB = (int16_t)(offsetrrB - adc_buffer.rrB);
  curR_phaC = (int16_t)(offsetrrC - adc_buffer.rrC);
  curR_DC   = (int16_t)(offsetdcr - adc_buffer.dcr);

  // Disable PWM when current limit is reached (current chopping)
  // This is the Level 2 of current protection. The Level 1 should kick in first given by I_MOT_MAX
  if(ABS(curL_DC) > curDC_max || enable == 0) {
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
  } else {
    LEFT_TIM->BDTR |= TIM_BDTR_MOE;
  }

  if(ABS(curR_DC)  > curDC_max || enable == 0) {
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
  } else {
    RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
  }

  // Create square wave for buzzer and more
  buzzerTimer++;
#ifdef BUZZER_ENABLED
  if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) {
    if (buzzerPrev == 0) {
      buzzerPrev = 1;
      if (++buzzerIdx > (buzzerCount + 2)) {    // pause 2 periods
        buzzerIdx = 1;
      }
    }
    if (buzzerTimer % buzzerFreq == 0 && (buzzerIdx <= buzzerCount || buzzerCount == 0)) {
      HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
    }
  } else if (buzzerPrev) {
      HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
      buzzerPrev = 0;
  }
#endif

  // Adjust pwm_margin depending on the selected Control Type
  if (FocAdapter_GetParams(FOC_ADAPTER_MOTOR_LEFT)->z_ctrlTypSel == FOC_CTRL) {
    pwm_margin = 110;
  } else {
    pwm_margin = 0;
  }

  // ############################### MOTOR CONTROL ###############################

  int ul, vl, wl;
  int ur, vr, wr;
  static boolean_T OverrunFlag = false;
  static uint32_t stallRecoverAtMs = 0;
  static uint8_t stallRecoveryActive = 0;

  /* Check for overrun */
  if (OverrunFlag) {
    return;
  }
  OverrunFlag = true;

  const uint32_t tickNowMs = HAL_GetTick();
  const uint8_t leftErrRaw = FocAdapter_GetErrorCode(FOC_ADAPTER_MOTOR_LEFT);
  const uint8_t rightErrRaw = FocAdapter_GetErrorCode(FOC_ADAPTER_MOTOR_RIGHT);
  const uint8_t stallErrActive = ((leftErrRaw & STALL_ERR_BIT) != 0U) || ((rightErrRaw & STALL_ERR_BIT) != 0U);

  if (!stallRecoveryActive && stallErrActive) {
    stallRecoveryActive = 1U;
    stallRecoverAtMs = tickNowMs + STALL_RECOVERY_DELAY_MS;
  }

  const uint8_t stallGraceActive = stallRecoveryActive && (stallRecoverAtMs > tickNowMs);

  /* During stall recovery we reject control input and keep motors disabled. */
  if (stallRecoveryActive) {
    enableFin = 0;
  } else {
    enableFin = enable && !leftErrRaw && !rightErrRaw;
  }
 
  // ========================= LEFT MOTOR ============================ 
    // Get hall sensors values
    uint8_t hall_ul = !(LEFT_HALL_U_PORT->IDR & LEFT_HALL_U_PIN);
    uint8_t hall_vl = !(LEFT_HALL_V_PORT->IDR & LEFT_HALL_V_PIN);
    uint8_t hall_wl = !(LEFT_HALL_W_PORT->IDR & LEFT_HALL_W_PIN);

    const FocAdapterInputFrame leftInputFrame = {
      .motorEnable = enableFin,
      .controlModeRequest = ctrlModReq,
      .inputTarget = pwml,
      .hallA = hall_ul,
      .hallB = hall_vl,
      .hallC = hall_wl,
      .phaseCurrentAB = curL_phaA,
      .phaseCurrentBC = curL_phaB,
      .dcLinkCurrent = curL_DC,
    };

    FocAdapter_SetInputFrame(FOC_ADAPTER_MOTOR_LEFT, &leftInputFrame);

    /* Step the controller */
    #ifdef MOTOR_LEFT_ENA
    FocAdapter_Step(FOC_ADAPTER_MOTOR_LEFT);
    #endif

    /* Get motor outputs here */
    const ExtY *leftOutput = FocAdapter_GetOutput(FOC_ADAPTER_MOTOR_LEFT);
    ul            = leftOutput->DC_phaA;
    vl            = leftOutput->DC_phaB;
    wl            = leftOutput->DC_phaC;

    const uint8_t inputIsNeutral = (ABS(pwml) <= STALL_NEUTRAL_PWM_DEADBAND) && (ABS(pwmr) <= STALL_NEUTRAL_PWM_DEADBAND);
    const uint8_t stallRecoveryReady = stallRecoveryActive && !stallGraceActive && inputIsNeutral;

    if (stallRecoveryReady) {
      FocAdapter_ClearErrorBits(STALL_ERR_BIT);
    }

    g_errCodeLeftEffective = FocAdapter_GetErrorCode(FOC_ADAPTER_MOTOR_LEFT);
    g_errCodeRightEffective = FocAdapter_GetErrorCode(FOC_ADAPTER_MOTOR_RIGHT);

    if (stallRecoveryReady) {
      g_errCodeLeftEffective &= (uint8_t)~STALL_ERR_BIT;
      g_errCodeRightEffective &= (uint8_t)~STALL_ERR_BIT;
    }

    /* Exit recovery only after raw stall bit is gone, preventing immediate retrigger loops. */
    if (stallRecoveryActive && !stallGraceActive && !stallErrActive) {
      stallRecoveryActive = 0U;
      stallRecoverAtMs = 0U;
    }

    /* Apply commands */
    LEFT_TIM->LEFT_TIM_U    = (uint16_t)CLAMP(ul + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    LEFT_TIM->LEFT_TIM_V    = (uint16_t)CLAMP(vl + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    LEFT_TIM->LEFT_TIM_W    = (uint16_t)CLAMP(wl + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
  // =================================================================
  

  // ========================= RIGHT MOTOR ===========================  
    // Get hall sensors values
    uint8_t hall_ur = !(RIGHT_HALL_U_PORT->IDR & RIGHT_HALL_U_PIN);
    uint8_t hall_vr = !(RIGHT_HALL_V_PORT->IDR & RIGHT_HALL_V_PIN);
    uint8_t hall_wr = !(RIGHT_HALL_W_PORT->IDR & RIGHT_HALL_W_PIN);

    const FocAdapterInputFrame rightInputFrame = {
      .motorEnable = enableFin,
      .controlModeRequest = ctrlModReq,
      .inputTarget = pwmr,
      .hallA = hall_ur,
      .hallB = hall_vr,
      .hallC = hall_wr,
      .phaseCurrentAB = curR_phaB,
      .phaseCurrentBC = curR_phaC,
      .dcLinkCurrent = curR_DC,
    };

    FocAdapter_SetInputFrame(FOC_ADAPTER_MOTOR_RIGHT, &rightInputFrame);

    /* Step the controller */
    #ifdef MOTOR_RIGHT_ENA
    FocAdapter_Step(FOC_ADAPTER_MOTOR_RIGHT);
    #endif

    /* Get motor outputs here */
    const ExtY *rightOutput = FocAdapter_GetOutput(FOC_ADAPTER_MOTOR_RIGHT);
    ur            = rightOutput->DC_phaA;
    vr            = rightOutput->DC_phaB;
    wr            = rightOutput->DC_phaC;

    /* Apply commands */
    RIGHT_TIM->RIGHT_TIM_U  = (uint16_t)CLAMP(ur + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    RIGHT_TIM->RIGHT_TIM_V  = (uint16_t)CLAMP(vr + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    RIGHT_TIM->RIGHT_TIM_W  = (uint16_t)CLAMP(wr + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
  // =================================================================

  /* Indicate task complete */
  OverrunFlag = false;
 
 // ###############################################################################

}
