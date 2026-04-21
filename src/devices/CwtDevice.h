#ifndef CWT_DEVICE_H
#define CWT_DEVICE_H

#include "IModbusDevice.h"
#include "../core/AppDataTypes.h"
#include <mbed.h>
#include <stdint.h>

class CwtDevice : public IModbusDevice {
 public:
  static const uint16_t kStartRegister = 0x0000;
  static const uint16_t kRegisterCount = 2;

  void setSlaveId(uint8_t id) { slaveId = id; }
  uint8_t getSlaveId() const { return slaveId; }

  ModbusReadConfig getReadConfig() const override {
    ModbusReadConfig config;
    config.slaveId = slaveId;
    config.startRegister = kStartRegister;
    config.registerCount = kRegisterCount;
    config.registerKind = ModbusRegisterKind::Holding;
    return config;
  }

  void reset() {
    mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
    snapshot = CwtSnapshot{};
  }

  bool updateFromRegisters(const uint16_t* regs,
                           uint16_t registerCount,
                           unsigned long nowMs) override {
    if (regs == nullptr || registerCount < 2U) {
      return false;
    }
    mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
    snapshot.valid = true;
    snapshot.rhPct = static_cast<float>(regs[0]) * 0.1f;
    snapshot.tempC = static_cast<float>(static_cast<int16_t>(regs[1])) * 0.1f;
    snapshot.lastUpdateMs = nowMs;
    return true;
  }

  void markInvalid() override {
    mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
    snapshot.valid = false;
  }

  CwtSnapshot getSnapshot() const {
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
  uint8_t slaveId = 0U;
  mutable rtos::Mutex stateMutex;
  CwtSnapshot snapshot;
};

#endif  // CWT_DEVICE_H
