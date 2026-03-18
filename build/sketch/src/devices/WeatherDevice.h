#line 1 "/home/billy/Documents/Code/iris-controls-v2/src/devices/WeatherDevice.h"
#ifndef WEATHER_DEVICE_H
#define WEATHER_DEVICE_H

#include "../core/SharedState.h"
#include <stdint.h>

class WeatherDevice {
 public:
  void reset() { snapshot = WeatherSnapshot{}; }

  bool updateFromRegisters(const uint16_t* regs, uint16_t registerCount, unsigned long nowMs) {
    if (regs == nullptr || registerCount < 8U) {
      return false;
    }

    const int32_t windRaw =
        (static_cast<int32_t>(regs[0]) << 16) | static_cast<int32_t>(regs[1]);
    const int32_t rainRaw =
        (static_cast<int32_t>(regs[6]) << 16) | static_cast<int32_t>(regs[7]);

    snapshot.valid = true;
    snapshot.windSpeedMps = static_cast<float>(windRaw) / 1000.0f;
    snapshot.rainDetected = (static_cast<float>(rainRaw) / 1000.0f) > 0.0f;
    snapshot.lastUpdateMs = nowMs;
    return true;
  }

  void updateFromSnapshot(const WeatherSnapshot& latest) { snapshot = latest; }

  void markInvalid() { snapshot.valid = false; }

  WeatherSnapshot getSnapshot() const { return snapshot; }

 private:
  WeatherSnapshot snapshot;
};

#endif  // WEATHER_DEVICE_H
