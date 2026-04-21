#ifndef MODBUS_SERVICE_H
#define MODBUS_SERVICE_H

#include "../config/SystemConfig.h"
#include "../devices/IModbusDevice.h"
#include <atomic>
#include <mbed.h>

struct DeviceRegistry;

class ModbusService {
 public:
  enum class DeviceRole : uint8_t {
    Weather = 0U,
    Cwt = 1U,
  };

  struct Health {
    bool busReady = false;
    /** True only after worker thread started successfully; used to avoid false watchdog when begin() fails. */
    bool pollThreadRunning = false;
    bool degraded = false;
    unsigned long lastLoopMs = 0UL;
    unsigned long totalReadFailures = 0UL;
    unsigned long weatherConsecutiveFailures = 0UL;
    unsigned long cwtConsecutiveFailures = 0UL;
  };

  /** Load commissioned devices from registry, init RS485/Modbus, start poll worker. */
  void begin(DeviceRegistry& registry);

  Health getHealth() const;

 private:
  struct PollEntry {
    IModbusDevice* device = nullptr;
    DeviceRole role = DeviceRole::Cwt;
    uint8_t deviceIndex = 0U;
    unsigned long pollIntervalMs = 0UL;
    unsigned long lastPollMs = 0UL;
    uint8_t consecutiveReadFailures = 0U;
  };

  static const uint8_t kMaxDevices = 8U;
  static const uint16_t kMaxRegistersPerPoll = 16U;

  void runThread();
  bool addPollEntry(IModbusDevice& device,
                    DeviceRole role,
                    unsigned long pollIntervalMs,
                    uint8_t deviceIndex);
  void pollDevices(unsigned long nowMs);
  void recordPollSuccess(DeviceRole role);
  void recordPollFailure(DeviceRole role);
  bool readRegisters(const ModbusReadConfig& config, uint16_t* outRegisters);
  bool readRegistersWithRetry(const ModbusReadConfig& config,
                              uint16_t* outRegisters,
                              int* diagResponseCount,
                              int* diagAvailableCount);
  bool readRegistersByKind(ModbusRegisterKind kind,
                           uint8_t slaveId,
                           uint16_t startRegister,
                           uint16_t registerCount,
                           uint16_t* outRegisters);
  void tryBusReset(unsigned long nowMs);
  void touchHeartbeat();
  bool isDegradedLocked() const;

  rtos::Thread workerThread{osPriorityNormal, SystemConfig::kModbusThreadStackSize};
  bool threadStarted = false;
  std::atomic<unsigned long> lastLoopMs{0UL};
  mutable rtos::Mutex stateMutex;
  PollEntry pollEntries[kMaxDevices];
  uint8_t pollEntryCount = 0U;
  uint8_t pollCursor = 0U;
  bool busReady = false;
  unsigned long totalReadFailures = 0UL;
  unsigned long weatherConsecutiveFailures = 0UL;
  unsigned long cwtConsecutiveFailures = 0UL;
  unsigned long lastBusResetMs = 0UL;
};

#endif  // MODBUS_SERVICE_H
