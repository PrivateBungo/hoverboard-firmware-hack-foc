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
#include "openloop_debug.h"

// Matlab includes and defines - from auto-code generation
// ###############################################################################
#include "BLDC_controller.h"           /* Model's header file */
#include "rtwtypes.h"

/* Forward declaration for Simulink lookup table helper (defined in BLDC_controller.c).
 * Signature must match: uint8_T plook_u8s16_evencka(int16_T u, int16_T bp0, uint16_T bpSpace, uint32_T maxIndex) */
uint8_T plook_u8s16_evencka(int16_T u, int16_T bp0, uint16_T bpSpace, uint32_T maxIndex);

extern RT_MODEL *const rtM_Left;
extern RT_MODEL *const rtM_Right;

extern DW   rtDW_Left;                  /* Observable states */
extern ExtU rtU_Left;                   /* External inputs */
extern ExtY rtY_Left;                   /* External outputs */
extern P    rtP_Left;

extern DW   rtDW_Right;                 /* Observable states */
extern ExtU rtU_Right;                  /* External inputs */
extern ExtY rtY_Right;                  /* External outputs */
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

#ifdef OPENLOOP_ENABLE
/* Compile-time guards to catch zero-duration misconfiguration that would cause division by zero */
_Static_assert(OPENLOOP_ALIGN_DURATION > 0,  "OPENLOOP_ALIGN_DURATION must be > 0");
_Static_assert(OPENLOOP_ACCEL_DURATION > 0,  "OPENLOOP_ACCEL_DURATION must be > 0");

typedef struct {
  int16_t  theta;           /* Current synthetic electrical angle [0, 23039] */
  int16_t  voltage;         /* Current voltage amplitude (ramping up) */
  int16_t  delta_theta;     /* Current angle increment per ISR cycle (ramping up) */
  uint16_t counter;         /* Cycle counter for timing phases */
  uint8_t  phase;           /* 0=inactive, 1=align, 2=rotate */
} OpenLoopState;

static OpenLoopState olStateL = {0};
static OpenLoopState olStateR = {0};
#endif

// =================================
// DMA interrupt frequency =~ 16 kHz
// =================================
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
  if (rtP_Left.z_ctrlTypSel == FOC_CTRL) {
    pwm_margin = 110;
  } else {
    pwm_margin = 0;
  }

  // ############################### MOTOR CONTROL ###############################

  int ul, vl, wl;
  int ur, vr, wr;
  static boolean_T OverrunFlag = false;

  /* Check for overrun */
  if (OverrunFlag) {
    return;
  }
  OverrunFlag = true;

  /* Make sure to stop BOTH motors in case of an error */
  enableFin = enable && !rtY_Left.z_errCode && !rtY_Right.z_errCode;
 
  // ========================= LEFT MOTOR ============================ 
    // Get hall sensors values
    uint8_t hall_ul = !(LEFT_HALL_U_PORT->IDR & LEFT_HALL_U_PIN);
    uint8_t hall_vl = !(LEFT_HALL_V_PORT->IDR & LEFT_HALL_V_PIN);
    uint8_t hall_wl = !(LEFT_HALL_W_PORT->IDR & LEFT_HALL_W_PIN);

    /* Set motor inputs here */
    rtU_Left.b_motEna     = enableFin;
    rtU_Left.z_ctrlModReq = ctrlModReq;  
    rtU_Left.r_inpTgt     = pwml;
    rtU_Left.b_hallA      = hall_ul;
    rtU_Left.b_hallB      = hall_vl;
    rtU_Left.b_hallC      = hall_wl;
    rtU_Left.i_phaAB      = curL_phaA;
    rtU_Left.i_phaBC      = curL_phaB;
    rtU_Left.i_DCLink     = curL_DC;
    // rtU_Left.a_mechAngle   = ...; // Angle input in DEGREES [0,360] in fixdt(1,16,4) data type. If `angle` is float use `= (int16_t)floor(angle * 16.0F)` If `angle` is integer use `= (int16_t)(angle << 4)`
    
    /* Step the controller */
    #ifdef MOTOR_LEFT_ENA    
    BLDC_controller_step(rtM_Left);
    #endif

    /* Get motor outputs here */
    ul            = rtY_Left.DC_phaA;
    vl            = rtY_Left.DC_phaB;
    wl            = rtY_Left.DC_phaC;
  // errCodeLeft  = rtY_Left.z_errCode;
  // motSpeedLeft = rtY_Left.n_mot;
  // motAngleLeft = rtY_Left.a_elecAngle;

#ifdef OPENLOOP_ENABLE
    /* Open-loop sinusoidal startup override for left motor */
    if (olStateL.phase == 0 && enableFin && ABS(pwml) > 50 && !rtDW_Left.n_commDeacv_Mode) {
      /* Start open-loop: initialise angle from current hall sector */
      int16_t hallValue_l = (int16_t)((hall_ul << 2) | (hall_vl << 1) | hall_wl);
      int8_t  sector_l    = rtConstP.vec_hallToPos_Value[hallValue_l];
      olStateL.theta       = (int16_t)(sector_l * 3840 + 1920);
      olStateL.voltage     = 0;
      olStateL.delta_theta = 0;
      olStateL.counter     = 0;
      olStateL.phase       = 1;
    }

    if (olStateL.phase > 0) {
      if (!enableFin || ABS(pwml) <= 50 || rtDW_Left.n_commDeacv_Mode) {
        /* Exit condition: FOC active, motor disabled, or command removed */
        olStateL.phase       = 0;
        olStateL.counter     = 0;
        olStateL.voltage     = 0;
        olStateL.delta_theta = 0;
      } else {
        olStateL.counter++;

        if (olStateL.phase == 1) {
          /* ALIGN phase: ramp voltage, hold static angle */
          olStateL.voltage = (int16_t)((int32_t)OPENLOOP_VOLTAGE_MAX * olStateL.counter / OPENLOOP_ALIGN_DURATION);
          if (olStateL.counter >= OPENLOOP_ALIGN_DURATION) {
            olStateL.voltage = OPENLOOP_VOLTAGE_MAX;
            olStateL.phase   = 2;
            olStateL.counter = 0;
          }
        } else {
          /* ROTATE phase: ramp delta_theta and advance synthetic angle */
          int16_t dt = (int16_t)((int32_t)OPENLOOP_DELTA_THETA_MAX * olStateL.counter / OPENLOOP_ACCEL_DURATION);
          if (dt > OPENLOOP_DELTA_THETA_MAX) {
            dt = OPENLOOP_DELTA_THETA_MAX;
          }
          olStateL.delta_theta = (pwml > 0) ? dt : (int16_t)(-dt);

          olStateL.theta += olStateL.delta_theta;
          if (olStateL.theta >= 23040) olStateL.theta -= 23040;
          if (olStateL.theta < 0)      olStateL.theta += 23040;

          if (olStateL.counter > OPENLOOP_ACCEL_DURATION) {
            olStateL.counter = OPENLOOP_ACCEL_DURATION;
          }
        }

        /* Generate 3-phase sinusoidal output from synthetic angle */
        uint8_T idx_l = plook_u8s16_evencka(olStateL.theta, 0, 128U, 180U);
        int16_t phaA_l = (int16_t)(((int32_t)olStateL.voltage * rtConstP.r_sin3PhaA_M1_Table[idx_l]) >> 14);
        int16_t phaB_l = (int16_t)(((int32_t)olStateL.voltage * rtConstP.r_sin3PhaB_M1_Table[idx_l]) >> 14);
        int16_t phaC_l = (int16_t)(((int32_t)olStateL.voltage * rtConstP.r_sin3PhaC_M1_Table[idx_l]) >> 14);

        /* Scale to match DC_phaA/B/C range and override controller outputs */
        ul = phaA_l >> 4;
        vl = phaB_l >> 4;
        wl = phaC_l >> 4;
        pwm_margin = 110;  /* same FOC ADC measurement window margin as when FOC_CTRL is active */
      }
    }
#endif

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

    /* Set motor inputs here */
    rtU_Right.b_motEna      = enableFin;
    rtU_Right.z_ctrlModReq  = ctrlModReq;
    rtU_Right.r_inpTgt      = pwmr;
    rtU_Right.b_hallA       = hall_ur;
    rtU_Right.b_hallB       = hall_vr;
    rtU_Right.b_hallC       = hall_wr;
    rtU_Right.i_phaAB       = curR_phaB;
    rtU_Right.i_phaBC       = curR_phaC;
    rtU_Right.i_DCLink      = curR_DC;
    // rtU_Right.a_mechAngle   = ...; // Angle input in DEGREES [0,360] in fixdt(1,16,4) data type. If `angle` is float use `= (int16_t)floor(angle * 16.0F)` If `angle` is integer use `= (int16_t)(angle << 4)`
    
    /* Step the controller */
    #ifdef MOTOR_RIGHT_ENA
    BLDC_controller_step(rtM_Right);
    #endif

    /* Get motor outputs here */
    ur            = rtY_Right.DC_phaA;
    vr            = rtY_Right.DC_phaB;
    wr            = rtY_Right.DC_phaC;
 // errCodeRight  = rtY_Right.z_errCode;
 // motSpeedRight = rtY_Right.n_mot;
 // motAngleRight = rtY_Right.a_elecAngle;

#ifdef OPENLOOP_ENABLE
    /* Open-loop sinusoidal startup override for right motor */
    if (olStateR.phase == 0 && enableFin && ABS(pwmr) > 50 && !rtDW_Right.n_commDeacv_Mode) {
      /* Start open-loop: initialise angle from current hall sector */
      int16_t hallValue_r = (int16_t)((hall_ur << 2) | (hall_vr << 1) | hall_wr);
      int8_t  sector_r    = rtConstP.vec_hallToPos_Value[hallValue_r];
      olStateR.theta       = (int16_t)(sector_r * 3840 + 1920);
      olStateR.voltage     = 0;
      olStateR.delta_theta = 0;
      olStateR.counter     = 0;
      olStateR.phase       = 1;
    }

    if (olStateR.phase > 0) {
      if (!enableFin || ABS(pwmr) <= 50 || rtDW_Right.n_commDeacv_Mode) {
        /* Exit condition: FOC active, motor disabled, or command removed */
        olStateR.phase       = 0;
        olStateR.counter     = 0;
        olStateR.voltage     = 0;
        olStateR.delta_theta = 0;
      } else {
        olStateR.counter++;

        if (olStateR.phase == 1) {
          /* ALIGN phase: ramp voltage, hold static angle */
          olStateR.voltage = (int16_t)((int32_t)OPENLOOP_VOLTAGE_MAX * olStateR.counter / OPENLOOP_ALIGN_DURATION);
          if (olStateR.counter >= OPENLOOP_ALIGN_DURATION) {
            olStateR.voltage = OPENLOOP_VOLTAGE_MAX;
            olStateR.phase   = 2;
            olStateR.counter = 0;
          }
        } else {
          /* ROTATE phase: ramp delta_theta and advance synthetic angle */
          int16_t dt = (int16_t)((int32_t)OPENLOOP_DELTA_THETA_MAX * olStateR.counter / OPENLOOP_ACCEL_DURATION);
          if (dt > OPENLOOP_DELTA_THETA_MAX) {
            dt = OPENLOOP_DELTA_THETA_MAX;
          }
          olStateR.delta_theta = (pwmr > 0) ? dt : (int16_t)(-dt);

          olStateR.theta += olStateR.delta_theta;
          if (olStateR.theta >= 23040) olStateR.theta -= 23040;
          if (olStateR.theta < 0)      olStateR.theta += 23040;

          if (olStateR.counter > OPENLOOP_ACCEL_DURATION) {
            olStateR.counter = OPENLOOP_ACCEL_DURATION;
          }
        }

        /* Generate 3-phase sinusoidal output from synthetic angle */
        uint8_T idx_r = plook_u8s16_evencka(olStateR.theta, 0, 128U, 180U);
        int16_t phaA_r = (int16_t)(((int32_t)olStateR.voltage * rtConstP.r_sin3PhaA_M1_Table[idx_r]) >> 14);
        int16_t phaB_r = (int16_t)(((int32_t)olStateR.voltage * rtConstP.r_sin3PhaB_M1_Table[idx_r]) >> 14);
        int16_t phaC_r = (int16_t)(((int32_t)olStateR.voltage * rtConstP.r_sin3PhaC_M1_Table[idx_r]) >> 14);

        /* Scale to match DC_phaA/B/C range and override controller outputs */
        ur = phaA_r >> 4;
        vr = phaB_r >> 4;
        wr = phaC_r >> 4;
        pwm_margin = 110;  /* same FOC ADC measurement window margin as when FOC_CTRL is active */
      }
    }
#endif

    /* Apply commands */
    RIGHT_TIM->RIGHT_TIM_U  = (uint16_t)CLAMP(ur + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    RIGHT_TIM->RIGHT_TIM_V  = (uint16_t)CLAMP(vr + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    RIGHT_TIM->RIGHT_TIM_W  = (uint16_t)CLAMP(wr + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
  // =================================================================

  /* Indicate task complete */
  OverrunFlag = false;
 
 // ###############################################################################

}

#ifdef OPENLOOP_ENABLE
/**
 * openloop_get_snapshot() -- atomically copy open-loop state for both motors.
 *
 * Interrupts are disabled for the duration of the copy to prevent torn reads
 * from the 16 kHz DMA ISR that updates olStateL/olStateR.
 */
void openloop_get_snapshot(OpenLoopSnapshot *left, OpenLoopSnapshot *right) {
  uint32_t prim = __get_PRIMASK();
  __disable_irq();
  left->phase        = (int16_t)olStateL.phase;
  left->theta        = olStateL.theta;
  left->delta_theta  = olStateL.delta_theta;
  left->voltage      = olStateL.voltage;
  right->phase       = (int16_t)olStateR.phase;
  right->theta       = olStateR.theta;
  right->delta_theta = olStateR.delta_theta;
  right->voltage     = olStateR.voltage;
  if (!prim) { __enable_irq(); }
}
#endif /* OPENLOOP_ENABLE */
