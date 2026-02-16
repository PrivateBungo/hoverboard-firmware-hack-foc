#include "input_decode.h"

InputDecodePair InputDecode_BuildPair(int16_t raw1,
                                      int16_t raw2,
                                      int16_t cmd1,
                                      int16_t cmd2) {
  InputDecodePair pair;

  pair.raw1 = raw1;
  pair.raw2 = raw2;
  pair.cmd1 = cmd1;
  pair.cmd2 = cmd2;

  return pair;
}
