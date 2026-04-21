#line 1 "/home/billy/Documents/Code/iris-controls-v2/src/devices/WeatherDevice.h"
#ifndef WEATHER_DEVICE_H
#define WEATHER_DEVICE_H

#include "IModbusDevice.h"
#include "../core/AppDataTypes.h"
#include <mbed.h>
#include <stdint.h>

class WeatherDevice : public IModbusDevice {
 public:
  static const uint8_t kSlaveId = 20U;
  static const uint16_t kStartRegister = 0x0012U;
  static const uint16_t kRegisterCount = 8U;

  ModbusReadConfig getReadConfig() const override {
    ModbusReadConfig config;
    config.slaveId = kSlaveId;
    config.startRegister = kStartRegister;
    config.registerCount = kRegisterCount;
    config.registerKind = ModbusRegisterKind::Input;
    return config;
  }

  void reset() {
    mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
    snapshot = WeatherSnapshot{};
  }

  bool updateFromRegisters(const uint16_t* regs,
                           uint16_t registerCount,
                           unsigned long nowMs) override {
    if (regs == nullptr || registerCount < 8U) {
      return false;
    }
    mbed::ScopedLock<rtos::Mutex> lock(stateMutex);

    const uint32_t windRawUnsigned =
        (static_cast<uint32_t>(regs[0]) << 16) | static_cast<uint32_t>(regs[1]);
    const uint32_t rainRawUnsigned =
        (static_cast<uint32_t>(regs[6]) << 16) | static_cast<uint32_t>(regs[7]);
    const int32_t windRaw = static_cast<int32_t>(windRawUnsigned);
    const int32_t rainRaw = static_cast<int32_t>(rainRawUnsigned);

    snapshot.valid = true;
    snapshot.windSpeedMps = static_cast<float>(windRaw) / 1000.0f;
    snapshot.rainDetected = (static_cast<float>(rainRaw) / 1000.0f) > 0.0f;
    snapshot.lastUpdateMs = nowMs;
    return true;
  }

  void markInvalid() override {
    mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
    snapshot.valid = false;
  }

  WeatherSnapshot getSnapshot() const {
    mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
    return snapshot;
  }

  bool isFresh(unsigned long nowMs, unsigned long maxAgeMs) const {
    mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
    if (!snapshot.valid || snapshot.lastUpdateMs == 0UL || nowMs < snapshot.lastUpdateMs) {
      return false;
    }
    return (nowMs - snapshot.lastUpdateMs) <= maxAgeMs;
  }

 private:
  mutable rtos::Mutex stateMutex;
  WeatherSnapshot snapshot;
};

#endif  // WEATHER_DEVICE_H
