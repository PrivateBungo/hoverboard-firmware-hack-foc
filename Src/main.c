/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
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

#include <stdio.h>
#include <stdlib.h> // for abs()
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "util.h"
#include "BLDC_controller.h"      /* BLDC's header file */
#include "rtwtypes.h"
#include "comms.h"
#include "drive_control.h"
#include "command_filter.h"
#include "intent_state_machine.h"
#include "user_intent.h"
#include "velocity_setpoint_layer.h"
#include "motor_controller.h"
#include "input_supervisor.h"
#include "mode_supervisor.h"
#include "stall_supervisor.h"
#include "wheel_command_supervisor.h"
#include "uart_reporting.h"
#include "input_decode.h"
#include "foc_adapter.h"

#if defined(DEBUG_I2C_LCD) || defined(SUPPORT_LCD)
#include "hd44780.h"
#endif

void SystemClock_Config(void);

#if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
#define DEBUG_SETPOINT_PRINT_INTERVAL_LOOPS  40U  // ~200 ms at 5 ms loop

static const char *IntentModeToString(IntentStateMachineMode mode) {
  switch (mode) {
    case INTENT_STATE_MACHINE_DRIVE_FORWARD:
      return "FWD";
    case INTENT_STATE_MACHINE_DRIVE_REVERSE:
      return "REV";
    case INTENT_STATE_MACHINE_ZERO_LATCH:
      return "ZL";
    default:
      return "UNK";
  }
}
#endif

//------------------------------------------------------------------------
// Global variables set externally
//------------------------------------------------------------------------
extern TIM_HandleTypeDef htim_left;
extern TIM_HandleTypeDef htim_right;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern volatile adc_buf_t adc_buffer;
#if defined(DEBUG_I2C_LCD) || defined(SUPPORT_LCD)
  extern LCD_PCF8574_HandleTypeDef lcd;
  extern uint8_t LCDerrorFlag;
#endif

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

volatile uint8_t uart_buf[200];

// Matlab defines - from auto-code generation
//---------------
extern P    rtP_Left;                   /* Block parameters (auto storage) */
extern P    rtP_Right;                  /* Block parameters (auto storage) */
//---------------

extern uint8_t     inIdx;               // input index used for dual-inputs
extern uint8_t     inIdx_prev;
extern InputStruct input1[];            // input structure
extern InputStruct input2[];            // input structure

extern int16_t speedAvg;                // Average measured speed
extern int16_t speedAvgAbs;             // Average measured speed in absolute
extern volatile uint32_t timeoutCntGen; // Timeout counter for the General timeout (PPM, PWM, Nunchuk)
extern volatile uint8_t  timeoutFlgGen; // Timeout Flag for the General timeout (PPM, PWM, Nunchuk)
extern uint8_t timeoutFlgADC;           // Timeout Flag for for ADC Protection: 0 = OK, 1 = Problem detected (line disconnected or wrong ADC data)
extern uint8_t timeoutFlgSerial;        // Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)

extern volatile int pwml;               // global variable for pwm left. -1000 to 1000
extern volatile int pwmr;               // global variable for pwm right. -1000 to 1000

extern uint8_t enable;                  // global variable for motor enable
extern uint8_t ctrlModReq;              // global variable for final control mode request

extern int16_t batVoltage;              // global variable for battery voltage
extern int16_t curL_DC;                  // ISR-sampled left DC link current ADC units
extern int16_t curR_DC;                  // ISR-sampled right DC link current ADC units

#if defined(SIDEBOARD_SERIAL_USART2)
extern SerialSideboard Sideboard_L;
#endif
#if defined(SIDEBOARD_SERIAL_USART3)
extern SerialSideboard Sideboard_R;
#endif
#if (defined(CONTROL_PPM_LEFT) && defined(DEBUG_SERIAL_USART3)) || (defined(CONTROL_PPM_RIGHT) && defined(DEBUG_SERIAL_USART2))
extern volatile uint16_t ppm_captured_value[PPM_NUM_CHANNELS+1];
#endif
#if (defined(CONTROL_PWM_LEFT) && defined(DEBUG_SERIAL_USART3)) || (defined(CONTROL_PWM_RIGHT) && defined(DEBUG_SERIAL_USART2))
extern volatile uint16_t pwm_captured_ch1_value;
extern volatile uint16_t pwm_captured_ch2_value;
#endif


//------------------------------------------------------------------------
// Global variables set here in main.c
//------------------------------------------------------------------------
uint8_t backwardDrive;
extern volatile uint32_t buzzerTimer;
volatile uint32_t main_loop_counter;
int16_t batVoltageCalib;         // global variable for calibrated battery voltage
int16_t board_temp_deg_c;        // global variable for calibrated temperature in degrees Celsius
int16_t left_dc_curr;            // global variable for Left DC Link current 
int16_t right_dc_curr;           // global variable for Right DC Link current
int16_t dc_curr;                 // global variable for Total DC Link current 
int16_t cmdL;                    // global variable for Left Command 
int16_t cmdR;                    // global variable for Right Command 

//------------------------------------------------------------------------
// Local variables
//------------------------------------------------------------------------
#if defined(FEEDBACK_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART3)
static UartReportingFrame feedbackFrame;
static UartReportingState uartReportingState;
#endif
#if defined(FEEDBACK_SERIAL_USART2)
static uint8_t sideboard_leds_L;
#endif
#if defined(FEEDBACK_SERIAL_USART3)
static uint8_t sideboard_leds_R;
#endif

#ifdef VARIANT_TRANSPOTTER
  uint8_t  nunchuk_connected;
  extern float    setDistance;  

  static uint8_t  checkRemote = 0;
  static uint16_t distance;
  static float    steering;
  static int      distanceErr;  
  static int      lastDistance = 0;
  static uint16_t transpotter_counter = 0;
#endif

static int16_t    speed;                // local variable for speed. -1000 to 1000
#ifndef VARIANT_TRANSPOTTER
  static int16_t  steer;                // local variable for steering. -1000 to 1000
  static UserIntentState userIntentState;
  static CommandFilterState commandFilterState;
  static IntentStateMachineState intentStateMachineState;
  static VelocitySetpointLayerState velocitySetpointLayerState;
  static MotorControllerState motorControllerState;
  static DriveControlStallDecayState stallDecayStateLeft;
  static DriveControlStallDecayState stallDecayStateRight;
  static WheelCommandSupervisorState wheelCommandSupervisorState;
  static InputSupervisorState inputSupervisorState;
  static ModeSupervisorState modeSupervisorState;
  static StallSupervisorState stallSupervisorState;
#endif

static uint32_t    buzzerTimer_prev = 0;
static uint32_t    inactivity_timeout_counter;
static MultipleTap MultipleTapBrake;    // define multiple tap functionality for the Brake pedal

#if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
  #define DEBUG_INPUT_PRINT_PERIOD_MS       5000U
  #define DEBUG_INPUT_PRINT_INTERVAL_LOOPS  (((DEBUG_INPUT_PRINT_PERIOD_MS / (DELAY_IN_MAIN_LOOP + 1U)) > 0U) ? (DEBUG_INPUT_PRINT_PERIOD_MS / (DELAY_IN_MAIN_LOOP + 1U)) : 1U)
  #define DEBUG_STALL_PRINT_PERIOD_MS        250U
  #define DEBUG_STALL_PRINT_INTERVAL_LOOPS  (((DEBUG_STALL_PRINT_PERIOD_MS / (DELAY_IN_MAIN_LOOP + 1U)) > 0U) ? (DEBUG_STALL_PRINT_PERIOD_MS / (DELAY_IN_MAIN_LOOP + 1U)) : 1U)
#endif

static uint16_t rate = RATE; // Adjustable rate to support multiple drive modes on startup

#define STALL_ERR_BIT (0x04U)

#ifdef MULTI_MODE_DRIVE
  static uint8_t drive_mode;
  static uint16_t max_speed;
#endif


int main(void) {

  HAL_Init();
  __HAL_RCC_AFIO_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  SystemClock_Config();

  __HAL_RCC_DMA1_CLK_DISABLE();
  MX_GPIO_Init();
  MX_TIM_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  BLDC_Init();        // BLDC Controller Init

  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);   // Activate Latch
  Input_Lim_Init();   // Input Limitations Init
  Input_Init();       // Input Init

  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);

  poweronMelody();
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

  #ifndef VARIANT_TRANSPOTTER
    UserIntent_Init(&userIntentState);
    CommandFilter_Init(&commandFilterState);
    IntentStateMachine_Init(&intentStateMachineState);
    VelocitySetpointLayer_Init(&velocitySetpointLayerState);
    MotorController_Init(&motorControllerState);
    DriveControl_ResetStallDecay(&stallDecayStateLeft);
    DriveControl_ResetStallDecay(&stallDecayStateRight);
    WheelCommandSupervisor_Init(&wheelCommandSupervisorState);
    InputSupervisor_Init(&inputSupervisorState);
    ModeSupervisor_Init(&modeSupervisorState, ctrlModReq);
    StallSupervisor_Init(&stallSupervisorState);
  #endif

  #ifndef VARIANT_TRANSPOTTER
    int16_t longitudinalRawCmd = 0;
    int16_t longitudinalCenteredCmd = 0;
    int16_t commandFilterLongitudinalOffsetOut = 0;
    uint8_t commandFilterLongitudinalCalibActive = 0U;
    uint8_t commandFilterLongitudinalCalibLocked = 0U;
    uint8_t commandFilterLongitudinalCalibUpdated = 0U;
    uint8_t commandFilterLongitudinalCalibInhibitTorque = 0U;
    int16_t userIntentLongitudinalOut = 0;
    int16_t intentVelocityOut = 0;
    int16_t intentCmdEffOut = 0;
    int8_t intentArmedSignOut = 0;
    int8_t intentBlockedSignOut = 0;
    uint8_t intentNearZeroOut = 0U;
    uint8_t intentModeOut = 0U;
    uint16_t intentZeroLatchElapsedMsOut = 0U;
    uint8_t intentZeroLatchReleasedOut = 0U;
    uint8_t intentZeroLatchArmedOut = 0U;
    uint8_t intentZeroLatchActivatedOut = 0U;
    int16_t setpointVelocityOut = 0;
    int16_t setpointAccelerationOut = 0;
    uint8_t setpointSlipGapClampActive = 0U;
    int16_t motorControllerSpeedErrorOut = 0;
    uint8_t motorControllerSaturatedOut = 0U;
    int16_t cmdL_adj = 0;
    int16_t cmdR_adj = 0;
  #endif

  #if defined(FEEDBACK_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART3)
    UartReporting_Init(&uartReportingState);
  #endif
  
  int32_t board_temp_adcFixdt = adc_buffer.temp << 16;  // Fixed-point filter output initialized with current ADC converted to fixed-point
  int16_t board_temp_adcFilt  = adc_buffer.temp;

  #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
    uint8_t prevLeftErrCode    = g_errCodeLeftEffective;
    uint8_t prevRightErrCode   = g_errCodeRightEffective;
    uint8_t prevTimeoutFlgADC  = timeoutFlgADC;
    uint8_t prevTimeoutFlgSerial = timeoutFlgSerial;
    uint8_t prevTimeoutFlgGen  = timeoutFlgGen;
    uint8_t prevEnableState    = enable;
    uint8_t prevCtrlModReq     = ctrlModReq;
    uint8_t prevIntentModeOut  = (uint8_t)INTENT_STATE_MACHINE_DRIVE_FORWARD;
    uint16_t prevIntentZeroLatchElapsedMsOut = 0U;
    uint8_t prevCommandFilterLongitudinalCalibActive = 0U;
    uint8_t prevCommandFilterLongitudinalCalibLocked = 0U;
    uint8_t prevCommandFilterLongitudinalCalibInhibitTorque = 0U;
    #ifndef VARIANT_TRANSPOTTER
      uint8_t prevStallActiveLeft  = 0U;
      uint8_t prevStallActiveRight = 0U;
    #endif

    printf("StallDecay cfg: spd<=%u trig>=%u preemptMs=%u preemptCmd=%u floor=%u totalMs=%u loopMs=%u ctrlModReq=%u\r\n",
      (unsigned)STALL_DECAY_SPEED_RPM,
      (unsigned)STALL_DECAY_CMD_TRIGGER,
      (unsigned)STALL_DECAY_PREEMPT_MS,
      (unsigned)STALL_DECAY_CMD_PREEMPT,
      (unsigned)STALL_DECAY_CMD_FLOOR,
      (unsigned)STALL_DECAY_TIME_MS,
      (unsigned)(DELAY_IN_MAIN_LOOP + 1U),
      (unsigned)CTRL_MOD_REQ);
    printf("StallDecay mode flags: inTRQ=%u inVLT=%u runtimeCtrlMode=%u\r\n",
      (unsigned)STALL_DECAY_IN_TRQ_MODE,
      (unsigned)STALL_DECAY_IN_VLT_MODE,
      (unsigned)ctrlModReq);
  #endif

  #ifdef MULTI_MODE_DRIVE
    if (adc_buffer.l_tx2 > input1[0].min + 50 && adc_buffer.l_rx2 > input2[0].min + 50) {
      drive_mode = 2;
      max_speed = MULTI_MODE_DRIVE_M3_MAX;
      rate = MULTI_MODE_DRIVE_M3_RATE;
      rtP_Left.n_max = rtP_Right.n_max = MULTI_MODE_M3_N_MOT_MAX << 4;
      rtP_Left.i_max = rtP_Right.i_max = (MULTI_MODE_M3_I_MOT_MAX * A2BIT_CONV) << 4;
    } else if (adc_buffer.l_tx2 > input1[0].min + 50) {
      drive_mode = 1;
      max_speed = MULTI_MODE_DRIVE_M2_MAX;
      rate = MULTI_MODE_DRIVE_M2_RATE;
      rtP_Left.n_max = rtP_Right.n_max = MULTI_MODE_M2_N_MOT_MAX << 4;
      rtP_Left.i_max = rtP_Right.i_max = (MULTI_MODE_M2_I_MOT_MAX * A2BIT_CONV) << 4;
    } else {
      drive_mode = 0;
      max_speed = MULTI_MODE_DRIVE_M1_MAX;
      rate = MULTI_MODE_DRIVE_M1_RATE;
      rtP_Left.n_max = rtP_Right.n_max = MULTI_MODE_M1_N_MOT_MAX << 4;
      rtP_Left.i_max = rtP_Right.i_max = (MULTI_MODE_M1_I_MOT_MAX * A2BIT_CONV) << 4;
    }

    printf("Drive mode %i selected: max_speed:%i acc_rate:%i \r\n", drive_mode, max_speed, rate);
  #endif

  // Loop until button is released
  while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) { HAL_Delay(10); }

  #ifdef MULTI_MODE_DRIVE
    // Wait until triggers are released. Exit if timeout elapses (to unblock if the inputs are not calibrated)
    int iTimeout = 0;
    while((adc_buffer.l_rx2 + adc_buffer.l_tx2) >= (input1[0].min + input2[0].min) && iTimeout++ < 300) {
      HAL_Delay(10);
    }
  #endif

  while(1) {
    if (buzzerTimer - buzzerTimer_prev > 16*DELAY_IN_MAIN_LOOP) {   // 1 ms = 16 ticks buzzerTimer

      /* User intent pipeline stage 1: raw input decode into user-facing command space. */
      readCommand();                        // Read Command: input1[inIdx].cmd, input2[inIdx].cmd
      calcAvgSpeed();                       // Calculate average measured speed: speedAvg, speedAvgAbs

    #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
      if (timeoutFlgADC != prevTimeoutFlgADC) {
        printf("SafetyFault ADC timeout %s (batADC:%i tempADC:%i)\r\n",
          (timeoutFlgADC != 0U) ? "ASSERT" : "CLEAR",
          (int)batVoltage,
          (int)adc_buffer.temp);
        prevTimeoutFlgADC = timeoutFlgADC;
      }

      if (timeoutFlgSerial != prevTimeoutFlgSerial) {
        printf("SafetyFault Serial timeout %s\r\n",
          (timeoutFlgSerial != 0U) ? "ASSERT" : "CLEAR");
        prevTimeoutFlgSerial = timeoutFlgSerial;
      }

      if (timeoutFlgGen != prevTimeoutFlgGen) {
        printf("SafetyFault General timeout %s\r\n",
          (timeoutFlgGen != 0U) ? "ASSERT" : "CLEAR");
        prevTimeoutFlgGen = timeoutFlgGen;
      }

      if (ctrlModReq != prevCtrlModReq) {
        printf("SafetyMode ctrlModReq %u -> %u (toADC:%u toSER:%u toGEN:%u)\r\n",
          (unsigned)prevCtrlModReq,
          (unsigned)ctrlModReq,
          (unsigned)timeoutFlgADC,
          (unsigned)timeoutFlgSerial,
          (unsigned)timeoutFlgGen);
        prevCtrlModReq = ctrlModReq;
      }

      if (enable != prevEnableState) {
        printf("SafetyState motors %s (ErrL:%u ErrR:%u toADC:%u toSER:%u toGEN:%u)\r\n",
          (enable != 0U) ? "ENABLED" : "DISABLED",
          (unsigned)g_errCodeLeftEffective,
          (unsigned)g_errCodeRightEffective,
          (unsigned)timeoutFlgADC,
          (unsigned)timeoutFlgSerial,
          (unsigned)timeoutFlgGen);
        prevEnableState = enable;
      }
    #endif

    #ifndef VARIANT_TRANSPOTTER
      InputSupervisor_Update(&inputSupervisorState, timeoutFlgADC, timeoutFlgSerial, timeoutFlgGen);
      ModeSupervisor_Select(&modeSupervisorState, ctrlModReq);

      // ####### MOTOR ENABLING: Only if the initial input is very small (for SAFETY) #######
      if (enable == 0 && ((g_errCodeLeftEffective & (uint8_t)~STALL_ERR_BIT) == 0U) && ((g_errCodeRightEffective & (uint8_t)~STALL_ERR_BIT) == 0U) &&
          ABS(input1[inIdx].cmd) < 50 && ABS(input2[inIdx].cmd) < 50){
        beepShort(6);                     // make 2 beeps indicating the motor enable
        beepShort(4); HAL_Delay(100);
        UserIntent_Reset(&userIntentState);
        CommandFilter_Reset(&commandFilterState);
        IntentStateMachine_Reset(&intentStateMachineState);
        VelocitySetpointLayer_Reset(&velocitySetpointLayerState);
        MotorController_Reset(&motorControllerState);
        DriveControl_ResetStallDecay(&stallDecayStateLeft);
        DriveControl_ResetStallDecay(&stallDecayStateRight);
        enable = 1;                       // enable motors
        #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
        printf("-- Motors enabled --\r\n");
        #endif
      }

      // ####### VARIANT_HOVERCAR #######
      #if defined(VARIANT_HOVERCAR) || defined(VARIANT_SKATEBOARD) || defined(ELECTRIC_BRAKE_ENABLE)
        uint16_t speedBlend;                                        // Calculate speed Blend, a number between [0, 1] in fixdt(0,16,15)
        speedBlend = (uint16_t)(((CLAMP(speedAvgAbs,10,60) - 10) << 15) / 50); // speedBlend [0,1] is within [10 rpm, 60rpm]
      #endif

      #ifdef STANDSTILL_HOLD_ENABLE
        standstillHold();                                           // Apply Standstill Hold functionality. Only available and makes sense for VOLTAGE or TORQUE Mode
      #endif

      #ifdef VARIANT_HOVERCAR
      if (inIdx == CONTROL_ADC) {                                   // Only use use implementation below if pedals are in use (ADC input)
        if (speedAvgAbs < 60) {                                     // Check if Hovercar is physically close to standstill to enable Double tap detection on Brake pedal for Reverse functionality
          multipleTapDet(input1[inIdx].cmd, HAL_GetTick(), &MultipleTapBrake); // Brake pedal in this case is "input1" variable
        }

        if (input1[inIdx].cmd > 30) {                               // If Brake pedal (input1) is pressed, bring to 0 also the Throttle pedal (input2) to avoid "Double pedal" driving
          input2[inIdx].cmd = (int16_t)((input2[inIdx].cmd * speedBlend) >> 15);
          cruiseControl((uint8_t)rtP_Left.b_cruiseCtrlEna);         // Cruise control deactivated by Brake pedal if it was active
        }
      }
      #endif

      #ifdef ELECTRIC_BRAKE_ENABLE
        electricBrake(speedBlend, MultipleTapBrake.b_multipleTap);  // Apply Electric Brake. Only available and makes sense for TORQUE Mode
      #endif

      #ifdef VARIANT_HOVERCAR
      if (inIdx == CONTROL_ADC) {                                   // Only use use implementation below if pedals are in use (ADC input)
        if (speedAvg > 0) {                                         // Make sure the Brake pedal is opposite to the direction of motion AND it goes to 0 as we reach standstill (to avoid Reverse driving by Brake pedal) 
          input1[inIdx].cmd = (int16_t)((-input1[inIdx].cmd * speedBlend) >> 15);
        } else {
          input1[inIdx].cmd = (int16_t)(( input1[inIdx].cmd * speedBlend) >> 15);
        }
      }
      #endif

      #ifdef VARIANT_SKATEBOARD
        if (input2[inIdx].cmd < 0) {                                // When Throttle is negative, it acts as brake. This condition is to make sure it goes to 0 as we reach standstill (to avoid Reverse driving) 
          if (speedAvg > 0) {                                       // Make sure the braking is opposite to the direction of motion
            input2[inIdx].cmd  = (int16_t)(( input2[inIdx].cmd * speedBlend) >> 15);
          } else {
            input2[inIdx].cmd  = (int16_t)((-input2[inIdx].cmd * speedBlend) >> 15);
          }
        }
      #endif

      // ####### COMMAND FILTER (longitudinal only troubleshooting path) #######
      {
        CommandFilterOutput commandFilterOutput;

        CommandFilter_Process(&commandFilterState,
                              input1[inIdx].cmd,
                              input2[inIdx].cmd,
                              &commandFilterOutput);

        commandFilterLongitudinalOffsetOut = commandFilterOutput.longitudinal_offset;
        commandFilterLongitudinalCalibActive = commandFilterOutput.longitudinal_calib_active;
        commandFilterLongitudinalCalibLocked = commandFilterOutput.longitudinal_calib_locked;
        commandFilterLongitudinalCalibUpdated = commandFilterOutput.longitudinal_calib_updated;
        commandFilterLongitudinalCalibInhibitTorque = commandFilterOutput.longitudinal_calib_inhibit_torque;

        longitudinalRawCmd = commandFilterOutput.longitudinal_raw;
        longitudinalCenteredCmd = commandFilterOutput.longitudinal_cmd;

        /*
         * Troubleshooting mode: bypass user-intent shaping and use only
         * longitudinal command directly as torque request.
         */
        steer = 0;
        speed = CLAMP(commandFilterOutput.longitudinal_cmd, -1000, 1000);
        userIntentLongitudinalOut = speed;
      }

      // ####### VARIANT_HOVERCAR #######
      #ifdef VARIANT_HOVERCAR
      if (inIdx == CONTROL_ADC) {               // Only use use implementation below if pedals are in use (ADC input)

        #ifdef MULTI_MODE_DRIVE
        if (speed >= max_speed) {
          speed = max_speed;
        }
        #endif

        if (!MultipleTapBrake.b_multipleTap) {  // Check driving direction
          speed = steer + speed;                // Forward driving: in this case steer = Brake, speed = Throttle
        } else {
          speed = steer - speed;                // Reverse driving: in this case steer = Brake, speed = Throttle
        }
        steer = 0;                              // Do not apply steering to avoid side effects if STEER_COEFFICIENT is NOT 0
      }
      #endif

      userIntentLongitudinalOut = speed;

      if (commandFilterLongitudinalCalibInhibitTorque != 0U) {
        /* Safety latch: suppress torque while boot-time neutral calibration is in progress. */
        steer = 0;
        speed = 0;
        UserIntent_Reset(&userIntentState);
        IntentStateMachine_Reset(&intentStateMachineState);
        VelocitySetpointLayer_Reset(&velocitySetpointLayerState);
        MotorController_Reset(&motorControllerState);
        DriveControl_ResetStallDecay(&stallDecayStateLeft);
        DriveControl_ResetStallDecay(&stallDecayStateRight);
        WheelCommandSupervisor_Init(&wheelCommandSupervisorState);
      }

      /*
       * Troubleshooting mode: disable intent/setpoint/motor PID/mixing layers.
       * Feed direct longitudinal torque command equally to both wheels.
       */
      intentVelocityOut = speed;
      intentCmdEffOut = speed;
      intentArmedSignOut = (speed > 0) - (speed < 0);
      intentBlockedSignOut = 0;
      intentNearZeroOut = (uint8_t)(ABS(speed) <= 10);
      intentModeOut = (uint8_t)INTENT_STATE_MACHINE_DRIVE_FORWARD;
      intentZeroLatchElapsedMsOut = 0U;
      intentZeroLatchReleasedOut = 0U;
      intentZeroLatchArmedOut = 0U;
      intentZeroLatchActivatedOut = 0U;
      setpointVelocityOut = speed;
      setpointAccelerationOut = 0;
      setpointSlipGapClampActive = 0U;
      motorControllerSpeedErrorOut = 0;
      motorControllerSaturatedOut = 0U;

      steer = 0;
      cmdL = speed;
      cmdR = speed;


      cmdL_adj = cmdL;
      cmdR_adj = cmdR;

      DriveControl_MapCommandsToPwm(cmdL_adj, cmdR_adj, &pwml, &pwmr);

      {
        int16_t pwmlBeforeDecay = (int16_t)pwml;
        int16_t pwmrBeforeDecay = (int16_t)pwmr;
        int16_t pwmlAfterDecay;
        int16_t pwmrAfterDecay;

        uint8_t stallDecayModeActive = ModeSupervisor_IsStallDecayActive(
          &modeSupervisorState,
          STALL_DECAY_IN_TRQ_MODE,
          TRQ_MODE,
          STALL_DECAY_IN_VLT_MODE,
          VLT_MODE);

        pwmlAfterDecay = DriveControl_ApplyStallDecay((int16_t)pwml, FocAdapter_GetMotorSpeed(FOC_ADAPTER_MOTOR_LEFT), stallDecayModeActive, &stallDecayStateLeft);
        pwmrAfterDecay = DriveControl_ApplyStallDecay((int16_t)pwmr, FocAdapter_GetMotorSpeed(FOC_ADAPTER_MOTOR_RIGHT), stallDecayModeActive, &stallDecayStateRight);
        pwml = pwmlAfterDecay;
        pwmr = pwmrAfterDecay;

        #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
          uint8_t stallActiveLeft  = (stallDecayStateLeft.stallTimerMs > 0U);
          uint8_t stallActiveRight = (stallDecayStateRight.stallTimerMs > 0U);
          uint8_t leftLimited = (ABS(pwmlBeforeDecay) > ABS(pwmlAfterDecay));
          uint8_t rightLimited = (ABS(pwmrBeforeDecay) > ABS(pwmrAfterDecay));

          if ((stallActiveLeft != prevStallActiveLeft) || (stallActiveRight != prevStallActiveRight)) {
            printf("StallDecay state L:%u(%ums) R:%u(%ums) nL:%i nR:%i cmdInL:%i cmdOutL:%i cmdInR:%i cmdOutR:%i\r\n",
              stallActiveLeft,
              stallDecayStateLeft.stallTimerMs,
              stallActiveRight,
              stallDecayStateRight.stallTimerMs,
              (int16_t)FocAdapter_GetMotorSpeed(FOC_ADAPTER_MOTOR_LEFT),
              (int16_t)FocAdapter_GetMotorSpeed(FOC_ADAPTER_MOTOR_RIGHT),
              pwmlBeforeDecay,
              pwmlAfterDecay,
              pwmrBeforeDecay,
              pwmrAfterDecay);
            prevStallActiveLeft = stallActiveLeft;
            prevStallActiveRight = stallActiveRight;
          }

          if ((stallActiveLeft || stallActiveRight) &&
              (main_loop_counter % DEBUG_STALL_PRINT_INTERVAL_LOOPS == 0U) &&
              (leftLimited || rightLimited)) {
            printf("StallDecay act tL:%ums tR:%ums nL:%i nR:%i inL:%i outL:%i inR:%i outR:%i limL:%u limR:%u\r\n",
              stallDecayStateLeft.stallTimerMs,
              stallDecayStateRight.stallTimerMs,
              (int16_t)FocAdapter_GetMotorSpeed(FOC_ADAPTER_MOTOR_LEFT),
              (int16_t)FocAdapter_GetMotorSpeed(FOC_ADAPTER_MOTOR_RIGHT),
              pwmlBeforeDecay,
              pwmlAfterDecay,
              pwmrBeforeDecay,
              pwmrAfterDecay,
              leftLimited,
              rightLimited);
          }
        #endif
      }

      {
        int16_t policyOutLeft;
        int16_t policyOutRight;
        uint8_t stallDriveEnable;

        StallSupervisor_Update(&stallSupervisorState,
                               HAL_GetTick(),
                               (int16_t)pwml,
                               (int16_t)pwmr,
                               (int16_t)FocAdapter_GetMotorSpeed(FOC_ADAPTER_MOTOR_LEFT),
                               (int16_t)FocAdapter_GetMotorSpeed(FOC_ADAPTER_MOTOR_RIGHT),
                               curL_DC,
                               curR_DC,
                               &policyOutLeft,
                               &policyOutRight,
                               &stallDriveEnable);

        pwml = policyOutLeft;
        pwmr = policyOutRight;

        if (stallDriveEnable == 0U) {
          enable = 0U;
        }
      }

    #endif

    #ifdef VARIANT_TRANSPOTTER
      distance    = CLAMP(input1[inIdx].cmd - 180, 0, 4095);
      steering    = (input2[inIdx].cmd - 2048) / 2048.0;
      distanceErr = distance - (int)(setDistance * 1345);

      if (nunchuk_connected == 0) {
        cmdL = cmdL * 0.8f + (CLAMP(distanceErr + (steering*((float)MAX(ABS(distanceErr), 50)) * ROT_P), -850, 850) * -0.2f);
        cmdR = cmdR * 0.8f + (CLAMP(distanceErr - (steering*((float)MAX(ABS(distanceErr), 50)) * ROT_P), -850, 850) * -0.2f);
        if (distanceErr > 0) {
          enable = 1;
        }
        if (distanceErr > -300) {
          #ifdef INVERT_R_DIRECTION
            pwmr = cmdR;
          #else
            pwmr = -cmdR;
          #endif
          #ifdef INVERT_L_DIRECTION
            pwml = -cmdL;
          #else
            pwml = cmdL;
          #endif

          if (checkRemote) {
            if (!HAL_GPIO_ReadPin(LED_PORT, LED_PIN)) {
              //enable = 1;
            } else {
              enable = 0;
            }
          }
        } else {
          enable = 0;
        }
        timeoutCntGen = 0;
        timeoutFlgGen = 0;
      }

      if (timeoutFlgGen) {
        pwml = 0;
        pwmr = 0;
        enable = 0;
        #ifdef SUPPORT_LCD
          LCD_SetLocation(&lcd,  0, 0); LCD_WriteString(&lcd, "Len:");
          LCD_SetLocation(&lcd,  8, 0); LCD_WriteString(&lcd, "m(");
          LCD_SetLocation(&lcd, 14, 0); LCD_WriteString(&lcd, "m)");
        #endif
        HAL_Delay(1000);
        nunchuk_connected = 0;
      }

      if ((distance / 1345.0) - setDistance > 0.5 && (lastDistance / 1345.0) - setDistance > 0.5) { // Error, robot too far away!
        enable = 0;
        beepLong(5);
        #ifdef SUPPORT_LCD
          LCD_ClearDisplay(&lcd);
          HAL_Delay(5);
          LCD_SetLocation(&lcd, 0, 0); LCD_WriteString(&lcd, "Emergency Off!");
          LCD_SetLocation(&lcd, 0, 1); LCD_WriteString(&lcd, "Keeper too fast.");
        #endif
        poweroff();
      }

      #ifdef SUPPORT_NUNCHUK
        if (transpotter_counter % 500 == 0) {
          if (nunchuk_connected == 0 && enable == 0) {
              if(Nunchuk_Read() == NUNCHUK_CONNECTED) {
                #ifdef SUPPORT_LCD
                  LCD_SetLocation(&lcd, 0, 0); LCD_WriteString(&lcd, "Nunchuk Control");
                #endif
                nunchuk_connected = 1;
	      }
	    } else {
              nunchuk_connected = 0;
	    }
          }
        }   
      #endif

      #ifdef SUPPORT_LCD
        if (transpotter_counter % 100 == 0) {
          if (LCDerrorFlag == 1 && enable == 0) {

          } else {
            if (nunchuk_connected == 0) {
              LCD_SetLocation(&lcd,  4, 0); LCD_WriteFloat(&lcd,distance/1345.0,2);
              LCD_SetLocation(&lcd, 10, 0); LCD_WriteFloat(&lcd,setDistance,2);
            }
            LCD_SetLocation(&lcd,  4, 1); LCD_WriteFloat(&lcd,batVoltage, 1);
            // LCD_SetLocation(&lcd, 11, 1); LCD_WriteFloat(&lcd,MAX(ABS(currentR), ABS(currentL)),2);
          }
        }
      #endif
      transpotter_counter++;
    #endif

    // ####### SIDEBOARDS HANDLING #######
    #if defined(SIDEBOARD_SERIAL_USART2)
      sideboardSensors((uint8_t)Sideboard_L.sensors);
    #endif
    #if defined(FEEDBACK_SERIAL_USART2)
      sideboardLeds(&sideboard_leds_L);
    #endif
    #if defined(SIDEBOARD_SERIAL_USART3)
      sideboardSensors((uint8_t)Sideboard_R.sensors);
    #endif
    #if defined(FEEDBACK_SERIAL_USART3)
      sideboardLeds(&sideboard_leds_R);
    #endif
    

    // ####### CALC BOARD TEMPERATURE #######
    filtLowPass32(adc_buffer.temp, TEMP_FILT_COEF, &board_temp_adcFixdt);
    board_temp_adcFilt  = (int16_t)(board_temp_adcFixdt >> 16);  // convert fixed-point to integer
    board_temp_deg_c    = (TEMP_CAL_HIGH_DEG_C - TEMP_CAL_LOW_DEG_C) * (board_temp_adcFilt - TEMP_CAL_LOW_ADC) / (TEMP_CAL_HIGH_ADC - TEMP_CAL_LOW_ADC) + TEMP_CAL_LOW_DEG_C;

    // ####### CALC CALIBRATED BATTERY VOLTAGE #######
    batVoltageCalib = batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC;

    // ####### CALC DC LINK CURRENT #######
    left_dc_curr  = -(FocAdapter_GetDcLinkCurrent(FOC_ADAPTER_MOTOR_LEFT) * 100) / A2BIT_CONV;   // Left DC Link Current * 100 
    right_dc_curr = -(FocAdapter_GetDcLinkCurrent(FOC_ADAPTER_MOTOR_RIGHT) * 100) / A2BIT_CONV;  // Right DC Link Current * 100
    dc_curr       = left_dc_curr + right_dc_curr;            // Total DC Link Current * 100

    // ####### DEBUG SERIAL OUT #######
    #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
      if (commandFilterLongitudinalCalibLocked != prevCommandFilterLongitudinalCalibLocked) {
        printf("CmdFilter calib lock:%u rawLong:%d longOff:%d centered:%d\r\n",
          (unsigned)commandFilterLongitudinalCalibLocked,
          longitudinalRawCmd,
          commandFilterLongitudinalOffsetOut,
          longitudinalCenteredCmd);
        prevCommandFilterLongitudinalCalibLocked = commandFilterLongitudinalCalibLocked;
      }

      if (commandFilterLongitudinalCalibInhibitTorque != prevCommandFilterLongitudinalCalibInhibitTorque) {
        printf("CmdFilter torque inhibit:%u rawLong:%d longOff:%d centered:%d\r\n",
          (unsigned)commandFilterLongitudinalCalibInhibitTorque,
          longitudinalRawCmd,
          commandFilterLongitudinalOffsetOut,
          longitudinalCenteredCmd);
        prevCommandFilterLongitudinalCalibInhibitTorque = commandFilterLongitudinalCalibInhibitTorque;
      }

      if (commandFilterLongitudinalCalibActive != prevCommandFilterLongitudinalCalibActive) {
        printf("CmdFilter calib active:%u rawLong:%d longOff:%d centered:%d\r\n",
          (unsigned)commandFilterLongitudinalCalibActive,
          longitudinalRawCmd,
          commandFilterLongitudinalOffsetOut,
          longitudinalCenteredCmd);
        prevCommandFilterLongitudinalCalibActive = commandFilterLongitudinalCalibActive;
      }

      if (commandFilterLongitudinalCalibUpdated != 0U) {
        printf("CmdFilter calib update rawLong:%d longOff:%d centered:%d\r\n",
          longitudinalRawCmd,
          commandFilterLongitudinalOffsetOut,
          longitudinalCenteredCmd);
      }

      if (intentModeOut != prevIntentModeOut) {
        printf("Intent mode transition: %u -> %u (cmd:%d speed:%d zLatchMs:%u)\r\n",
          (unsigned)prevIntentModeOut,
          (unsigned)intentModeOut,
          intentCmdEffOut,
          speedAvg,
          (unsigned)intentZeroLatchElapsedMsOut);
        prevIntentModeOut = intentModeOut;
      }

      if (intentZeroLatchArmedOut != 0U) {
        printf("Intent zero-latch armed sign:%d cmdEff:%d speed:%d nearZero:%u\r\n",
          (int)intentArmedSignOut,
          intentCmdEffOut,
          speedAvg,
          (unsigned)intentNearZeroOut);
      }

      if (intentZeroLatchActivatedOut != 0U) {
        printf("Intent zero-latch active blocked:%d cmdEff:%d speed:%d\r\n",
          (int)intentBlockedSignOut,
          intentCmdEffOut,
          speedAvg);
      }

      if (intentModeOut == (uint8_t)INTENT_STATE_MACHINE_ZERO_LATCH &&
          intentZeroLatchElapsedMsOut != prevIntentZeroLatchElapsedMsOut &&
          (intentZeroLatchElapsedMsOut % 100U) == 0U) {
        printf("Intent zero-latch timer: %u ms (cmd:%d speed:%d)\r\n",
          (unsigned)intentZeroLatchElapsedMsOut,
          intentCmdEffOut,
          speedAvg);
      }

      if (intentZeroLatchReleasedOut != 0U) {
        printf("Intent zero-latch release (cmdEff:%d speed:%d nearZero:%u)\r\n",
          intentCmdEffOut,
          speedAvg,
          (unsigned)intentNearZeroOut);
      }
      prevIntentZeroLatchElapsedMsOut = intentZeroLatchElapsedMsOut;

      if (g_errCodeLeftEffective != prevLeftErrCode || g_errCodeRightEffective != prevRightErrCode) {
        printf("MotorErr L:%u[b0:%u b1:%u b2:%u] R:%u[b0:%u b1:%u b2:%u]\r\n",
          g_errCodeLeftEffective,
          ((g_errCodeLeftEffective  & 0x01U) != 0U),
          ((g_errCodeLeftEffective  & 0x02U) != 0U),
          ((g_errCodeLeftEffective  & 0x04U) != 0U),
          g_errCodeRightEffective,
          ((g_errCodeRightEffective & 0x01U) != 0U),
          ((g_errCodeRightEffective & 0x02U) != 0U),
          ((g_errCodeRightEffective & 0x04U) != 0U));

        prevLeftErrCode  = g_errCodeLeftEffective;
        prevRightErrCode = g_errCodeRightEffective;
      }

      if (main_loop_counter % DEBUG_SETPOINT_PRINT_INTERVAL_LOOPS == 0U) {
        printf("SetpointTrace mode:%s rawLong:%d longOff:%d rawLongMinusOff:%d uCmd:%d cmdEff:%d arm:%d blk:%d nz:%u calibA:%u calibL:%u calibI:%u vIntent:%d vSet:%d aSet:%d vAct:%d vErr:%d vSat:%u slip:%u zLatchMs:%u zRel:%u\r\n",
          IntentModeToString((IntentStateMachineMode)intentModeOut),
          longitudinalRawCmd,
          commandFilterLongitudinalOffsetOut,
          longitudinalCenteredCmd,
          userIntentLongitudinalOut,
          intentCmdEffOut,
          (int)intentArmedSignOut,
          (int)intentBlockedSignOut,
          (unsigned)intentNearZeroOut,
          (unsigned)commandFilterLongitudinalCalibActive,
          (unsigned)commandFilterLongitudinalCalibLocked,
          (unsigned)commandFilterLongitudinalCalibInhibitTorque,
          intentVelocityOut,
          setpointVelocityOut,
          setpointAccelerationOut,
          speedAvg,
          motorControllerSpeedErrorOut,
          (unsigned)motorControllerSaturatedOut,
          (unsigned)setpointSlipGapClampActive,
          (unsigned)intentZeroLatchElapsedMsOut,
          (unsigned)intentZeroLatchReleasedOut);
      }

      if (main_loop_counter % DEBUG_INPUT_PRINT_INTERVAL_LOOPS == 0) {    // Send data periodically every ~5 s
        #if defined(DEBUG_SERIAL_PROTOCOL)
          process_debug();
        #else
          InputDecodePair inputDecodePair = InputDecode_BuildPair(input1[inIdx].raw, input2[inIdx].raw, cmdL, cmdR);
          printf("in1:%i in2:%i rawLong:%i longOff:%i rawMinusOff:%i uCmd:%i cmdEff:%i arm:%d blk:%d nz:%u calibA:%u calibL:%u calibI:%u vIntent:%i iMode:%u zLatchMs:%u zRel:%u vSp:%i aSp:%i vErr:%i vSat:%u slip:%u cmdL:%i cmdR:%i ErrL:%u ErrR:%u BatADC:%i BatV:%i TempADC:%i Temp:%i StallL_t:%u StallR_t:%u StallSup:%u CtrlMode:%u\r\n",
            inputDecodePair.raw1,     // 1: INPUT1
            inputDecodePair.raw2,     // 2: INPUT2
            longitudinalRawCmd,
            commandFilterLongitudinalOffsetOut,
            longitudinalCenteredCmd,
            userIntentLongitudinalOut,
            intentCmdEffOut,
            (int)intentArmedSignOut,
            (int)intentBlockedSignOut,
            (unsigned)intentNearZeroOut,
            (unsigned)commandFilterLongitudinalCalibActive,
            (unsigned)commandFilterLongitudinalCalibLocked,
            (unsigned)commandFilterLongitudinalCalibInhibitTorque,
            intentVelocityOut,
            (unsigned)intentModeOut,
            (unsigned)intentZeroLatchElapsedMsOut,
            (unsigned)intentZeroLatchReleasedOut,
            setpointVelocityOut,
            setpointAccelerationOut,
            motorControllerSpeedErrorOut,
            (unsigned)motorControllerSaturatedOut,
            (unsigned)setpointSlipGapClampActive,
            inputDecodePair.cmd1,
            inputDecodePair.cmd2,
            g_errCodeLeftEffective,       // 5: left motor error code flags
            g_errCodeRightEffective,      // 6: right motor error code flags
            adc_buffer.batt1,         // 7: for battery voltage calibration
            batVoltageCalib,          // 8: for verifying battery voltage calibration
            board_temp_adcFilt,       // 9: for board temperature calibration
            board_temp_deg_c,         // 10: for verifying board temperature calibration
            stallDecayStateLeft.stallTimerMs,
            stallDecayStateRight.stallTimerMs,
            (unsigned)StallSupervisor_IsActive(&stallSupervisorState),
            ctrlModReq);        // 10: for verifying board temperature calibration
        #endif
      }
    #endif

    // ####### FEEDBACK SERIAL OUT #######
    #if defined(FEEDBACK_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART3)
      if (main_loop_counter % 2 == 0) {    // Send data periodically every 10 ms
        #if defined(FEEDBACK_SERIAL_USART2)
          if(__HAL_DMA_GET_COUNTER(huart2.hdmatx) == 0) {
            UartReporting_PrepareFrame(&feedbackFrame,
                                       (uint16_t)SERIAL_START_FRAME,
                                       (int16_t)input1[inIdx].cmd,
                                       (int16_t)input2[inIdx].cmd,
                                       (int16_t)FocAdapter_GetMotorSpeed(FOC_ADAPTER_MOTOR_RIGHT),
                                       (int16_t)FocAdapter_GetMotorSpeed(FOC_ADAPTER_MOTOR_LEFT),
                                       (int16_t)batVoltageCalib,
                                       (int16_t)board_temp_deg_c,
                                       (uint16_t)sideboard_leds_L);
            UartReporting_OnFrame(&uartReportingState);
            HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&feedbackFrame, sizeof(feedbackFrame));
          }
        #endif
        #if defined(FEEDBACK_SERIAL_USART3)
          if(__HAL_DMA_GET_COUNTER(huart3.hdmatx) == 0) {
            UartReporting_PrepareFrame(&feedbackFrame,
                                       (uint16_t)SERIAL_START_FRAME,
                                       (int16_t)input1[inIdx].cmd,
                                       (int16_t)input2[inIdx].cmd,
                                       (int16_t)FocAdapter_GetMotorSpeed(FOC_ADAPTER_MOTOR_RIGHT),
                                       (int16_t)FocAdapter_GetMotorSpeed(FOC_ADAPTER_MOTOR_LEFT),
                                       (int16_t)batVoltageCalib,
                                       (int16_t)board_temp_deg_c,
                                       (uint16_t)sideboard_leds_R);
            UartReporting_OnFrame(&uartReportingState);
            HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&feedbackFrame, sizeof(feedbackFrame));
          }
        #endif
      }
    #endif

    // ####### POWEROFF BY POWER-BUTTON #######
    poweroffPressCheck();

    // ####### BEEP AND EMERGENCY POWEROFF #######
    if (TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF && speedAvgAbs < 20){  // poweroff before mainboard burns OR low bat 3
      #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
        printf("Powering off, temperature is too high\r\n");
      #endif
      poweroff();
    } else if ( BAT_DEAD_ENABLE && batVoltage < BAT_DEAD && speedAvgAbs < 20){
      #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
        printf("Powering off, battery voltage is too low\r\n");
      #endif
      poweroff();
    } else if ((g_errCodeLeftEffective & (uint8_t)~STALL_ERR_BIT) || (g_errCodeRightEffective & (uint8_t)~STALL_ERR_BIT)) {                                           // 1 beep (low pitch): Motor error, disable motors
      enable = 0;
      beepCount(1, 24, 1);
    } else if (timeoutFlgADC) {                                                                       // 2 beeps (low pitch): ADC timeout
      beepCount(2, 24, 1);
    } else if (timeoutFlgSerial) {                                                                    // 3 beeps (low pitch): Serial timeout
      beepCount(3, 24, 1);
    } else if (timeoutFlgGen) {                                                                       // 4 beeps (low pitch): General timeout (PPM, PWM, Nunchuk)
      beepCount(4, 24, 1);
    } else if (TEMP_WARNING_ENABLE && board_temp_deg_c >= TEMP_WARNING) {                             // 5 beeps (low pitch): Mainboard temperature warning
      beepCount(5, 24, 1);
    } else if (BAT_LVL1_ENABLE && batVoltage < BAT_LVL1) {                                            // 1 beep fast (medium pitch): Low bat 1
      beepCount(0, 10, 6);
    } else if (BAT_LVL2_ENABLE && batVoltage < BAT_LVL2) {                                            // 1 beep slow (medium pitch): Low bat 2
      beepCount(0, 10, 30);
    } else if (BEEPS_BACKWARD && (((cmdR < -50 || cmdL < -50) && speedAvg < 0) || MultipleTapBrake.b_multipleTap)) { // 1 beep fast (high pitch): Backward spinning motors
      beepCount(0, 5, 1);
      backwardDrive = 1;
    } else {  // do not beep
      beepCount(0, 0, 0);
      backwardDrive = 0;
    }


    inactivity_timeout_counter++;

    // ####### INACTIVITY TIMEOUT #######
    if (abs(cmdL) > 50 || abs(cmdR) > 50) {
      inactivity_timeout_counter = 0;
    }

    #if defined(CRUISE_CONTROL_SUPPORT) || defined(STANDSTILL_HOLD_ENABLE)
      if ((abs(rtP_Left.n_cruiseMotTgt)  > 50 && rtP_Left.b_cruiseCtrlEna) || 
          (abs(rtP_Right.n_cruiseMotTgt) > 50 && rtP_Right.b_cruiseCtrlEna)) {
        inactivity_timeout_counter = 0;
      }
    #endif

    if (inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 60 * 1000) / (DELAY_IN_MAIN_LOOP + 1)) {  // rest of main loop needs maybe 1ms
      #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
        printf("Powering off, wheels were inactive for too long\r\n");
      #endif
      poweroff();
    }


    // HAL_GPIO_TogglePin(LED_PORT, LED_PIN);                 // This is to measure the main() loop duration with an oscilloscope connected to LED_PIN
    // Update states
    inIdx_prev = inIdx;
    buzzerTimer_prev = buzzerTimer;
    main_loop_counter++;
    }
  }
}


// ===========================================================
/** System Clock Configuration
*/
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType           = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource        = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider       = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider      = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider      = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection    = RCC_PERIPHCLK_ADC;
  // PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV8;  // 8 MHz
  PeriphClkInit.AdcClockSelection       = RCC_ADCPCLK2_DIV4;  // 16 MHz
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
