#include "diag_snapshot.h"

void DiagSnapshot_Init(DiagSnapshotState *state) {
  if (state == 0) {
    return;
  }
  state->snapshot_counter = 0U;
}

void DiagSnapshot_Capture(DiagSnapshotState *state) {
  if (state == 0) {
    return;
  }
  state->snapshot_counter++;
}
