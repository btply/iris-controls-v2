#ifndef IMODBUS_DEVICE_H
#define IMODBUS_DEVICE_H

#include <stdint.h>

enum class ModbusRegisterKind : uint8_t {
  Holding = 0U,
  Input = 1U,
};

struct ModbusReadConfig {
  uint8_t slaveId = 0U;
  uint16_t startRegister = 0U;
  uint16_t registerCount = 0U;
  ModbusRegisterKind registerKind = ModbusRegisterKind::Holding;
};

class IModbusDevice {
 public:
  virtual ~IModbusDevice() = default;

  virtual ModbusReadConfig getReadConfig() const = 0;
  virtual bool updateFromRegisters(const uint16_t* regs,
                                   uint16_t registerCount,
                                   unsigned long nowMs) = 0;
  virtual void markInvalid() = 0;
};

#endif  // IMODBUS_DEVICE_H
