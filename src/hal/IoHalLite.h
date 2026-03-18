#ifndef IO_HAL_LITE_H
#define IO_HAL_LITE_H

#include <stdint.h>

namespace IoHal {

struct Status {
  bool initialized;
  bool expansionBound;
  bool expansionConfigured;
  bool expansionWriteHealthy;
  bool expansionOutputStateUncertain;
  unsigned long expansionWriteFailures;
  unsigned long expansionConsecutiveFailures;
};

void begin();
void update();

bool writeSharedSignalLevel(bool high, bool enforceOutputInterlock = true);
bool writeOutput(uint8_t outputChannel, bool enabled, bool warnIfUnavailable = true);

void disableAllOutputs();
bool isAnyOutputEnabled();
bool isOutputAvailable(uint8_t outputChannel);
Status getStatus();

}  // namespace IoHal

#endif  // IO_HAL_LITE_H
