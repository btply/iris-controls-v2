#ifndef CWT_DEVICE_H
#define CWT_DEVICE_H

#include "IModbusDevice.h"
#include "../core/AppDataTypes.h"
#include <mbed.h>
#include <stdint.h>

class CwtDevice : public IModbusDevice {
 public:
  static const uint16_t kStartRegister = 0x0000U;
  static const uint16_t kRegisterCount = 2U;

  static uint8_t defaultSlaveIdForIndex(uint8_t index) {
    static const uint8_t kDefaultSlaveIds[AppDataConfig::kCwtCount] = {2U, 3U, 4U, 5U};
    if (index >= AppDataConfig::kCwtCount) {
      return 0U;
    }
    return kDefaultSlaveIds[index];
  }

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
    stateMutex.lock();
    snapshot = CwtSnapshot{};
    stateMutex.unlock();
  }

  bool updateFromRegisters(const uint16_t* regs,
                           uint16_t registerCount,
                           unsigned long nowMs) override {
    if (regs == nullptr || registerCount < 2U) {
      return false;
    }
    stateMutex.lock();
    snapshot.valid = true;
    snapshot.rhPct = static_cast<float>(regs[0]) * 0.1f;
    snapshot.tempC = static_cast<float>(static_cast<int16_t>(regs[1])) * 0.1f;
    snapshot.lastUpdateMs = nowMs;
    stateMutex.unlock();
    return true;
  }

  void markInvalid() override {
    stateMutex.lock();
    snapshot.valid = false;
    stateMutex.unlock();
  }

  CwtSnapshot getSnapshot() const {
    stateMutex.lock();
    const CwtSnapshot copy = snapshot;
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
  uint8_t slaveId = 0U;
  mutable rtos::Mutex stateMutex;
  CwtSnapshot snapshot;
};

#endif  // CWT_DEVICE_H
