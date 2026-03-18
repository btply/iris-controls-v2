#ifndef MODBUS_SERVICE_H
#define MODBUS_SERVICE_H

#include "../config/SystemConfig.h"
#include "../devices/IModbusDevice.h"
#include <atomic>
#include <mbed.h>

class ModbusService {
 public:
  enum class DeviceRole : uint8_t {
    Weather = 0U,
    Cwt = 1U,
  };

  struct Health {
    bool busReady = false;
    bool degraded = false;
    unsigned long lastLoopMs = 0UL;
    unsigned long totalReadFailures = 0UL;
    unsigned long weatherConsecutiveFailures = 0UL;
    unsigned long cwtConsecutiveFailures = 0UL;
  };

  void begin();
  bool registerDevice(IModbusDevice& device,
                      DeviceRole role,
                      unsigned long pollIntervalMs,
                      uint8_t deviceIndex = 0U);

  void start();
  void stop();

  Health getHealth() const;

 private:
  struct PollEntry {
    IModbusDevice* device = nullptr;
    DeviceRole role = DeviceRole::Cwt;
    uint8_t deviceIndex = 0U;
    unsigned long pollIntervalMs = 0UL;
    unsigned long lastPollMs = 0UL;
  };

  static const uint8_t kMaxDevices = 8U;
  static const uint16_t kMaxRegistersPerPoll = 16U;

  void runThread();
  void pollDevices(unsigned long nowMs);
  void recordPollSuccess(DeviceRole role);
  void recordPollFailure(DeviceRole role);
  bool ensureBusReady(unsigned long nowMs);
  bool readRegisters(const ModbusReadConfig& config, uint16_t* outRegisters);
  bool readHoldingRegisters(uint8_t slaveId,
                            uint16_t startRegister,
                            uint16_t registerCount,
                            uint16_t* outRegisters);
  bool readInputRegisters(uint8_t slaveId,
                          uint16_t startRegister,
                          uint16_t registerCount,
                          uint16_t* outRegisters);
  void touchHeartbeat();
  bool isDegradedLocked() const;

  rtos::Thread workerThread{osPriorityNormal, SystemConfig::kModbusThreadStackSize};
  std::atomic<bool> running{false};
  std::atomic<unsigned long> lastLoopMs{0UL};
  mutable rtos::Mutex stateMutex;
  PollEntry pollEntries[kMaxDevices];
  uint8_t pollEntryCount = 0U;
  uint8_t pollCursor = 0U;
  bool busReady = false;
  bool busReadyLogged = false;
  unsigned long lastBusInitAttemptMs = 0UL;
  unsigned long totalReadFailures = 0UL;
  unsigned long weatherConsecutiveFailures = 0UL;
  unsigned long cwtConsecutiveFailures = 0UL;
};

#endif  // MODBUS_SERVICE_H
