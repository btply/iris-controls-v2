#line 1 "/home/billy/Documents/Code/iris-controls-v2/src/devices/CwtDevice.h"
#ifndef CWT_DEVICE_H
#define CWT_DEVICE_H

#include "../core/SharedState.h"
#include <stdint.h>

class CwtDevice {
 public:
  void reset() { snapshot = CwtSnapshot{}; }

  bool updateFromRegisters(const uint16_t* regs, uint16_t registerCount, unsigned long nowMs) {
    if (regs == nullptr || registerCount < 2U) {
      return false;
    }
    snapshot.valid = true;
    snapshot.rhPct = static_cast<float>(regs[0]) * 0.1f;
    snapshot.tempC = static_cast<float>(static_cast<int16_t>(regs[1])) * 0.1f;
    snapshot.lastUpdateMs = nowMs;
    return true;
  }

  void updateFromSnapshot(const CwtSnapshot& latest) { snapshot = latest; }

  void markInvalid() { snapshot.valid = false; }

  CwtSnapshot getSnapshot() const { return snapshot; }

 private:
  CwtSnapshot snapshot;
};

#endif  // CWT_DEVICE_H
