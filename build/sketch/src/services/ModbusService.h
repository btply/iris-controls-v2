#line 1 "/home/billy/Documents/Code/iris-controls-v2/src/services/ModbusService.h"
#ifndef MODBUS_SERVICE_H
#define MODBUS_SERVICE_H

#include "../core/SharedState.h"
#include <atomic>
#include <mbed.h>

class ModbusService {
 public:
  struct Health {
    bool busReady = false;
    bool weatherFresh = false;
    bool cwtFresh = false;
    bool degraded = false;
    unsigned long lastLoopMs = 0UL;
    unsigned long lastWeatherSuccessMs = 0UL;
    unsigned long lastCwtSuccessMs = 0UL;
    unsigned long totalReadFailures = 0UL;
    unsigned long weatherConsecutiveFailures = 0UL;
    unsigned long cwtConsecutiveFailures = 0UL;
  };

  void begin(SharedState* sharedStateIn);

  void start();
  void stop();

  bool hasFreshData(unsigned long nowMs) const;
  Health getHealth(unsigned long nowMs) const;

 private:
  static const uint8_t kCwtSensorCount = SharedStateConfig::kCwtCount;
  static const uint8_t kWeatherSlaveId = 20U;
  static const uint8_t kCwtSlaveIds[kCwtSensorCount];
  static const bool kUseSyntheticDataIfBusUnavailable = false;

  void runThread();
  bool ensureBusReady(unsigned long nowMs);
  bool readHoldingRegisters(uint8_t slaveId,
                            uint16_t startRegister,
                            uint16_t registerCount,
                            uint16_t* outRegisters);
  bool readInputRegisters(uint8_t slaveId,
                          uint16_t startRegister,
                          uint16_t registerCount,
                          uint16_t* outRegisters);
  void readWeatherIfDue(unsigned long nowMs);
  void readCwtIfDue(unsigned long nowMs);
  bool isWeatherFreshLocked(unsigned long nowMs) const;
  bool isCwtFreshLocked(unsigned long nowMs) const;
  bool isDegradedLocked() const;

  bool readWeatherFromBus(WeatherSnapshot* out, unsigned long nowMs);
  bool readCwtFromBus(uint8_t sensorIndex, CwtSnapshot* out, unsigned long nowMs);
  void fillSyntheticWeather(WeatherSnapshot* out, unsigned long nowMs) const;
  void fillSyntheticCwt(uint8_t sensorIndex, CwtSnapshot* out, unsigned long nowMs) const;

  SharedState* sharedState = nullptr;
  rtos::Thread workerThread;
  std::atomic<bool> running{false};
  mutable rtos::Mutex stateMutex;
  bool busReady = false;
  bool busReadyLogged = false;
  unsigned long lastBusInitAttemptMs = 0UL;
  unsigned long lastLoopMs = 0UL;
  unsigned long lastWeatherSuccessMs = 0UL;
  unsigned long lastCwtSuccessMs = 0UL;
  unsigned long totalReadFailures = 0UL;
  unsigned long weatherConsecutiveFailures = 0UL;
  unsigned long cwtConsecutiveFailures = 0UL;
  unsigned long lastWeatherPollMs = 0UL;
  unsigned long lastCwtPollMs = 0UL;
};

#endif  // MODBUS_SERVICE_H
