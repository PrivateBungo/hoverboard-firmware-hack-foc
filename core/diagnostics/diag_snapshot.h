#ifndef DIAG_SNAPSHOT_H
#define DIAG_SNAPSHOT_H

#include <stdint.h>

typedef struct {
  uint32_t snapshot_counter;
} DiagSnapshotState;

void DiagSnapshot_Init(DiagSnapshotState *state);
void DiagSnapshot_Capture(DiagSnapshotState *state);

#endif /* DIAG_SNAPSHOT_H */
