#include "app_loop.h"

void AppLoop_Init(AppLoopState *state) {
  if (state == 0) {
    return;
  }
  state->tick_count = 0U;
}

void AppLoop_Tick(AppLoopState *state) {
  if (state == 0) {
    return;
  }
  state->tick_count++;
}
