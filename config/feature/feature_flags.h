#ifndef FEATURE_FLAGS_H
#define FEATURE_FLAGS_H

// ############################### VARIANT SELECTION ###############################
// PlatformIO: uncomment desired variant in platformio.ini
// Keil uVision: select desired variant from the Target drop down menu (to the right of the Load button)
// Ubuntu: define the desired build variant here if you want to use make in console
// or use VARIANT environment variable for example like "make -e VARIANT=VARIANT_NUNCHUK". Select only one at a time.
#if !defined(PLATFORMIO)
  //#define VARIANT_ADC         // Variant for control via ADC input
  //#define VARIANT_USART       // Variant for Serial control via USART3 input
  //#define VARIANT_NUNCHUK     // Variant for Nunchuk controlled vehicle build
  //#define VARIANT_PPM         // Variant for RC-Remote with PPM-Sum Signal
  #define VARIANT_PWM         // Variant for RC-Remote with PWM Signal
  //#define VARIANT_IBUS        // Variant for RC-Remotes with FLYSKY IBUS
  //#define VARIANT_HOVERCAR    // Variant for HOVERCAR build
  //#define VARIANT_HOVERBOARD  // Variant for HOVERBOARD build
  //#define VARIANT_TRANSPOTTER // Variant for TRANSPOTTER build https://github.com/NiklasFauth/hoverboard-firmware-hack/wiki/Build-Instruction:-TranspOtter https://hackaday.io/project/161891-transpotter-ng
  //#define VARIANT_SKATEBOARD  // Variant for SKATEBOARD build
#endif
// ########################### END OF VARIANT SELECTION ############################

// #define PRINTF_FLOAT_SUPPORT          // [-] Uncomment this for printf to support float on Serial Debug. It will increase code size! Better to avoid it!

#endif // FEATURE_FLAGS_H
