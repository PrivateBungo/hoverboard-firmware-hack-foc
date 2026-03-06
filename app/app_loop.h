#ifndef APP_LOOP_H
#define APP_LOOP_H

#include <stdint.h>

typedef struct {
  uint32_t tick_count;
} AppLoopState;

void AppLoop_Init(AppLoopState *state);
void AppLoop_Tick(AppLoopState *state);

#endif /* APP_LOOP_H */
