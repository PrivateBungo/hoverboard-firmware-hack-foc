#ifndef USER_PARAMS_H
#define USER_PARAMS_H

// ############################## DEFAULT SETTINGS ############################
// Default settings will be applied at the end of this config file if not set before
#define INACTIVITY_TIMEOUT        8       // Minutes of not driving until poweroff. it is not very precise.
#define BEEPS_BACKWARD            1       // 0 or 1
#define ADC_MARGIN                100     // ADC input margin applied on the raw ADC min and max to make sure the MIN and MAX values are reached even in the presence of noise
#define ADC_PROTECT_TIMEOUT       100     // ADC Protection: number of wrong / missing input commands before safety state is taken
#define ADC_PROTECT_THRESH        200     // ADC Protection threshold below/above the MIN/MAX ADC values
#define AUTO_CALIBRATION_ENA              // Enable/Disable input auto-calibration by holding power button pressed. Un-comment this if auto-calibration is not needed.

/* FILTER is in fixdt(0,16,16): VAL_fixedPoint = VAL_floatingPoint * 2^16. In this case 6553 = 0.1 * 2^16
 * Value of COEFFICIENT is in fixdt(1,16,14)
 * If VAL_floatingPoint >= 0, VAL_fixedPoint = VAL_floatingPoint * 2^14
 * If VAL_floatingPoint < 0,  VAL_fixedPoint = 2^16 + floor(VAL_floatingPoint * 2^14).
*/
// Value of RATE is in fixdt(1,16,4): VAL_fixedPoint = VAL_floatingPoint * 2^4. In this case 480 = 30 * 2^4
#define DEFAULT_RATE                480   // 30.0f [-] lower value == slower rate [0, 32767] = [0.0, 2047.9375]. Do NOT make rate negative (>32767)
#define DEFAULT_FILTER              6553  // Default for FILTER 0.1f [-] lower value == softer filter [0, 65535] = [0.0 - 1.0].
#define DEFAULT_SPEED_COEFFICIENT   16384 // Default for SPEED_COEFFICIENT 1.0f [-] higher value == stronger. [0, 65535] = [-2.0 - 2.0]. In this case 16384 = 1.0 * 2^14
#define DEFAULT_STEER_COEFFICIENT   8192  // Defualt for STEER_COEFFICIENT 0.5f [-] higher value == stronger. [0, 65535] = [-2.0 - 2.0]. In this case  8192 = 0.5 * 2^14. If you do not want any steering, set it to 0.
// ######################### END OF DEFAULT SETTINGS ##########################

#endif // USER_PARAMS_H
