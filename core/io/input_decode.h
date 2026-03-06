#ifndef INPUT_DECODE_H
#define INPUT_DECODE_H

#include <stdint.h>

typedef struct {
  int16_t raw1;
  int16_t raw2;
  int16_t cmd1;
  int16_t cmd2;
} InputDecodePair;

InputDecodePair InputDecode_BuildPair(int16_t raw1,
                                      int16_t raw2,
                                      int16_t cmd1,
                                      int16_t cmd2);

#endif /* INPUT_DECODE_H */
