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
    stateMutex.lock();
    snapshot = WeatherSnapshot{};
    stateMutex.unlock();
  }

  bool updateFromRegisters(const uint16_t* regs,
                           uint16_t registerCount,
                           unsigned long nowMs) override {
    if (regs == nullptr || registerCount < 8U) {
      return false;
    }
    stateMutex.lock();

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
    stateMutex.unlock();
    return true;
  }

  void markInvalid() override {
    stateMutex.lock();
    snapshot.valid = false;
    stateMutex.unlock();
  }

  WeatherSnapshot getSnapshot() const {
    stateMutex.lock();
    const WeatherSnapshot copy = snapshot;
    stateMutex.unlock();
    return copy;
  }

  bool isFresh(unsigned long nowMs, unsigned long maxAgeMs) const {
    stateMutex.lock();
    if (!snapshot.valid || snapshot.lastUpdateMs == 0UL || nowMs < snapshot.lastUpdateMs) {
      stateMutex.unlock();
      return false;
    }
    const bool fresh = (nowMs - snapshot.lastUpdateMs) <= maxAgeMs;
    stateMutex.unlock();
    return fresh;
  }

 private:
  mutable rtos::Mutex stateMutex;
  WeatherSnapshot snapshot;
};

#endif  // WEATHER_DEVICE_H
