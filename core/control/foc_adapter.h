#ifndef FOC_ADAPTER_H
#define FOC_ADAPTER_H

#include "BLDC_controller.h"

/*
 * Compatibility bridge for the future generated-code boundary.
 * Iteration 1 intentionally forwards directly to existing generated symbols.
 */
#define FocAdapter_LeftInput rtU_Left
#define FocAdapter_RightInput rtU_Right
#define FocAdapter_LeftOutput rtY_Left
#define FocAdapter_RightOutput rtY_Right

#endif /* FOC_ADAPTER_H */
