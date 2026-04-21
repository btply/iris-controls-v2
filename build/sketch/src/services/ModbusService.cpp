#line 1 "/home/billy/Documents/Code/iris-controls-v2/src/services/ModbusService.cpp"
#include "ModbusService.h"

#include "../config/SystemConfig.h"
#include "../core/DeviceRegistry.h"
#include "../core/AppDataTypes.h"
#include "../devices/ModbusCommissioningTable.h"
#include "LoggerService.h"
#include <Arduino.h>
#include <ArduinoModbus.h>
#include <ArduinoRS485.h>
#include <chrono>
#include <mbed.h>

bool ModbusService::addPollEntry(IModbusDevice& device,
                                 DeviceRole role,
                                 unsigned long pollIntervalMs,
                                 uint8_t deviceIndex) {
  if (pollIntervalMs == 0UL) {
    return false;
  }
  const ModbusReadConfig config = device.getReadConfig();
  if (config.slaveId == 0U || config.slaveId > 247U) {
    return false;
  }
  if (config.registerCount == 0U || config.registerCount > kMaxRegistersPerPoll) {
    return false;
  }
  const uint32_t endRegister = static_cast<uint32_t>(config.startRegister) +
                               static_cast<uint32_t>(config.registerCount) - 1UL;
  if (endRegister > 0xFFFFUL) {
    return false;
  }

  mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
  if (pollEntryCount >= kMaxDevices) {
    return false;
  }

  PollEntry& entry = pollEntries[pollEntryCount];
  const uint8_t slot = pollEntryCount;
  const unsigned long nowMs = millis();
  const unsigned long phaseSliceMs =
      (pollIntervalMs > kMaxDevices) ? (pollIntervalMs / kMaxDevices) : 0UL;
  const unsigned long phaseOffsetMs = phaseSliceMs * slot;
  entry.device = &device;
  entry.role = role;
  entry.deviceIndex = deviceIndex;
  entry.pollIntervalMs = pollIntervalMs;
  entry.lastPollMs = nowMs - pollIntervalMs + phaseOffsetMs;
  entry.consecutiveReadFailures = 0U;
  pollEntryCount++;
  return true;
}

void ModbusService::tryBusReset(unsigned long nowMs) {
  if (nowMs - lastBusResetMs < SystemConfig::kModbusBusResetMinIntervalMs) {
    return;
  }
  lastBusResetMs = nowMs;
  LoggerService::enqueue(LoggerService::Level::Warn, "ModbusService", "bus_reset_attempt");
  ModbusRTUClient.end();
  rtos::ThisThread::sleep_for(std::chrono::milliseconds(50));
  const bool ok = ModbusRTUClient.begin(RS485, 9600, SERIAL_8N1);
  if (!ok) {
    LoggerService::enqueue(LoggerService::Level::Error, "ModbusService", "bus_reset_begin_failed");
    {
      mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
      busReady = false;
    }
    return;
  }
  RS485.setDelays(SystemConfig::kRs485PreDelayUs, SystemConfig::kRs485PostDelayUs);
  ModbusRTUClient.setTimeout(SystemConfig::kModbusResponseTimeoutMs);
  {
    mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
    busReady = true;
  }
  LoggerService::enqueue(LoggerService::Level::Info, "ModbusService", "bus_reset_ok");
}

void ModbusService::begin(DeviceRegistry& registry) {
  if (threadStarted) {
    return;
  }

  {
    mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
    pollEntryCount = 0U;
    pollCursor = 0U;
    for (uint8_t i = 0; i < kMaxDevices; i++) {
      pollEntries[i] = PollEntry{};
    }
    totalReadFailures = 0UL;
    weatherConsecutiveFailures = 0UL;
    cwtConsecutiveFailures = 0UL;
    lastBusResetMs = 0UL;
    lastLoopMs.store(millis());
  }

  using ModbusCommissioning::Entry;
  using ModbusCommissioning::Kind;
  using ModbusCommissioning::kEntries;
  using ModbusCommissioning::kEntryCount;

  if (SystemConfig::kSkipWeatherPolling) {
    LoggerService::enqueue(LoggerService::Level::Info, "ModbusService", "weather_polling_disabled");
  }

  for (size_t e = 0; e < kEntryCount; e++) {
    const Entry& row = kEntries[e];
    if (row.kind == Kind::Weather) {
      registry.weather.reset();
      if (SystemConfig::kSkipWeatherPolling) {
        continue;
      }
      if (!addPollEntry(registry.weather,
                        DeviceRole::Weather,
                        SystemConfig::kWeatherPollIntervalMs,
                        0U)) {
        LoggerService::enqueue(LoggerService::Level::Error, "ModbusService", "register_weather_failed");
      } else {
        LoggerService::enqueue(LoggerService::Level::Info, "ModbusService", "registered_weather");
      }
      continue;
    }
    if (row.kind == Kind::Cwt) {
      if (row.cwtSlotIndex >= AppDataConfig::kCwtCount) {
        LoggerService::enqueuePrintf(LoggerService::Level::Error,
                                     "ModbusService",
                                     "commission_cwt_slot_oob idx=%u",
                                     static_cast<unsigned int>(row.cwtSlotIndex));
        continue;
      }
      registry.cwt[row.cwtSlotIndex].setSlaveId(row.slaveId);
      registry.cwt[row.cwtSlotIndex].reset();
      if (!addPollEntry(registry.cwt[row.cwtSlotIndex],
                        DeviceRole::Cwt,
                        SystemConfig::kCwtPollIntervalMs,
                        row.cwtSlotIndex)) {
        LoggerService::enqueuePrintf(LoggerService::Level::Error,
                                     "ModbusService",
                                     "register_cwt%u_failed",
                                     static_cast<unsigned int>(row.cwtSlotIndex));
      }
    }
  }

  const bool ok = ModbusRTUClient.begin(RS485, 9600, SERIAL_8N1);

  if (!ok) {
    LoggerService::enqueue(LoggerService::Level::Warn, "ModbusService", "modbus_begin_failed");
    {
      mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
      busReady = false;
    }
    return;
  }

  RS485.setDelays(SystemConfig::kRs485PreDelayUs, SystemConfig::kRs485PostDelayUs);
  ModbusRTUClient.setTimeout(SystemConfig::kModbusResponseTimeoutMs);
  LoggerService::enqueue(LoggerService::Level::Info, "ModbusService", "bus_ready");

  {
    mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
    busReady = true;
  }

  const osStatus status = workerThread.start(mbed::callback(this, &ModbusService::runThread));
  if (status != osOK) {
    {
      mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
      busReady = false;
    }
    LoggerService::enqueue(LoggerService::Level::Error, "ModbusService", "thread_start_failed");
    return;
  }

  threadStarted = true;
}

ModbusService::Health ModbusService::getHealth() const {
  Health health;
  mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
  health.busReady = busReady;
  health.pollThreadRunning = threadStarted;
  health.lastLoopMs = lastLoopMs.load();
  health.totalReadFailures = totalReadFailures;
  health.weatherConsecutiveFailures = weatherConsecutiveFailures;
  health.cwtConsecutiveFailures = cwtConsecutiveFailures;
  health.degraded = isDegradedLocked();
  return health;
}

bool ModbusService::isDegradedLocked() const {
  return weatherConsecutiveFailures >= SystemConfig::kModbusFailureDegradeThreshold ||
         cwtConsecutiveFailures >= SystemConfig::kModbusFailureDegradeThreshold;
}

void ModbusService::touchHeartbeat() {
  lastLoopMs.store(millis());
}

void ModbusService::runThread() {
  while (true) {
    touchHeartbeat();
    const unsigned long nowMs = millis();
    pollDevices(nowMs);
    touchHeartbeat();
    rtos::ThisThread::sleep_for(std::chrono::milliseconds(SystemConfig::kModbusLoopSleepMs));
  }
}

bool ModbusService::readRegistersByKind(ModbusRegisterKind kind,
                                        uint8_t slaveId,
                                        uint16_t startRegister,
                                        uint16_t registerCount,
                                        uint16_t* outRegisters) {
  if (outRegisters == nullptr || registerCount == 0U) {
    return false;
  }

  const int registerTable =
      (kind == ModbusRegisterKind::Input) ? INPUT_REGISTERS : HOLDING_REGISTERS;

  touchHeartbeat();
  const int responseCount =
      ModbusRTUClient.requestFrom(slaveId, registerTable, startRegister, registerCount);
  if (responseCount != static_cast<int>(registerCount)) {
    return false;
  }
  touchHeartbeat();
  for (uint16_t i = 0; i < registerCount; i++) {
    touchHeartbeat();
    if (!ModbusRTUClient.available()) {
      return false;
    }
    const long v = ModbusRTUClient.read();
    if (v < 0) {
      return false;
    }
    outRegisters[i] = static_cast<uint16_t>(v);
    touchHeartbeat();
  }
  return true;
}

bool ModbusService::readRegistersWithRetry(const ModbusReadConfig& config,
                                           uint16_t* outRegisters,
                                           int* diagResponseCount,
                                           int* diagAvailableCount) {
  const uint8_t maxAttempts =
      static_cast<uint8_t>(1U + static_cast<unsigned int>(SystemConfig::kModbusReadRetries));
  for (uint8_t attempt = 0U; attempt < maxAttempts; attempt++) {
    if (attempt > 0U) {
      rtos::ThisThread::sleep_for(std::chrono::milliseconds(SystemConfig::kModbusRetryDelayMs));
    }
    if (readRegisters(config, outRegisters)) {
      return true;
    }
    touchHeartbeat();
  }
  const int registerTable =
      (config.registerKind == ModbusRegisterKind::Input) ? INPUT_REGISTERS : HOLDING_REGISTERS;
  touchHeartbeat();
  const int resp =
      ModbusRTUClient.requestFrom(config.slaveId, registerTable,
                                  config.startRegister, config.registerCount);
  if (diagResponseCount != nullptr) {
    *diagResponseCount = resp;
  }
  if (diagAvailableCount != nullptr) {
    *diagAvailableCount = static_cast<int>(ModbusRTUClient.available());
  }
  return false;
}

bool ModbusService::readRegisters(const ModbusReadConfig& config, uint16_t* outRegisters) {
  if (outRegisters == nullptr) {
    return false;
  }
  if (config.slaveId == 0U || config.slaveId > 247U) {
    return false;
  }
  if (config.registerCount == 0U || config.registerCount > kMaxRegistersPerPoll) {
    return false;
  }
  const uint32_t endRegister = static_cast<uint32_t>(config.startRegister) +
                             static_cast<uint32_t>(config.registerCount) - 1UL;
  if (endRegister > 0xFFFFUL) {
    return false;
  }
  return readRegistersByKind(config.registerKind,
                             config.slaveId,
                             config.startRegister,
                             config.registerCount,
                             outRegisters);
}

void ModbusService::recordPollSuccess(DeviceRole role) {
  mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
  if (role == DeviceRole::Weather) {
    weatherConsecutiveFailures = 0UL;
  } else {
    cwtConsecutiveFailures = 0UL;
  }
}

void ModbusService::recordPollFailure(DeviceRole role) {
  mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
  totalReadFailures++;
  if (role == DeviceRole::Weather) {
    weatherConsecutiveFailures++;
  } else {
    cwtConsecutiveFailures++;
  }
}

void ModbusService::pollDevices(unsigned long nowMs) {
  uint8_t count = 0U;
  uint8_t startCursor = 0U;
  {
    mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
    count = pollEntryCount;
    startCursor = pollCursor;
  }
  if (count == 0U) {
    return;
  }

  for (uint8_t offset = 0; offset < count; offset++) {
    const uint8_t i = static_cast<uint8_t>((startCursor + offset) % count);
    PollEntry localEntry;
    bool due = false;
    {
      mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
      localEntry = pollEntries[i];
      if (localEntry.device != nullptr &&
          (nowMs - localEntry.lastPollMs) >= localEntry.pollIntervalMs) {
        due = true;
      }
    }
    if (!due) {
      continue;
    }

    const ModbusReadConfig readConfig = localEntry.device->getReadConfig();
    uint16_t regs[kMaxRegistersPerPoll] = {};
    int diagResp = -1;
    int diagAvail = -1;
    bool ok = readRegistersWithRetry(readConfig, regs, &diagResp, &diagAvail);
    if (ok) {
      ok = localEntry.device->updateFromRegisters(regs, readConfig.registerCount, nowMs);
    }

    if (!ok) {
      const char tableCode =
          (readConfig.registerKind == ModbusRegisterKind::Input) ? 'I' : 'H';
      const char* roleLabel =
          (localEntry.role == DeviceRole::Weather) ? "weather" : "cwt";
      const char* lastError = ModbusRTUClient.lastError();
      if (lastError == nullptr) {
        lastError = "none";
      }
      LoggerService::enqueuePrintf(LoggerService::Level::Warn,
                                   "ModbusService",
                                   "read_fail_diag role=%s idx=%u sid=%u tbl=%c reg=0x%04X cnt=%u "
                                   "resp=%d avail=%d err=%s",
                                   roleLabel,
                                   static_cast<unsigned int>(localEntry.deviceIndex),
                                   static_cast<unsigned int>(readConfig.slaveId),
                                   tableCode,
                                   static_cast<unsigned int>(readConfig.startRegister),
                                   static_cast<unsigned int>(readConfig.registerCount),
                                   diagResp,
                                   diagAvail,
                                   lastError);

      {
        uint8_t cf = 0U;
        {
          mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
          if (i < pollEntryCount) {
            pollEntries[i].consecutiveReadFailures++;
            cf = pollEntries[i].consecutiveReadFailures;
          }
        }
        if (cf >= SystemConfig::kModbusBusResetFailureThreshold) {
          tryBusReset(nowMs);
          mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
          if (i < pollEntryCount) {
            pollEntries[i].consecutiveReadFailures = 0U;
          }
        }
      }

      localEntry.device->markInvalid();
      recordPollFailure(localEntry.role);
      {
        mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
        if (i < pollEntryCount) {
          pollEntries[i].lastPollMs = nowMs;
        }
        pollCursor = static_cast<uint8_t>((i + 1U) % count);
      }
      touchHeartbeat();
      rtos::ThisThread::sleep_for(std::chrono::milliseconds(SystemConfig::kModbusInterRequestDelayMs));
      return;
    }

    {
      mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
      if (i < pollEntryCount) {
        pollEntries[i].consecutiveReadFailures = 0U;
      }
    }

    if (SystemConfig::kModbusLogSuccessfulReads) {
      if (localEntry.role == DeviceRole::Weather) {
        LoggerService::enqueuePrintf(LoggerService::Level::Info,
                                     "ModbusService",
                                     "read_ok_weather_s%u",
                                     static_cast<unsigned int>(readConfig.slaveId));
      } else {
        LoggerService::enqueuePrintf(LoggerService::Level::Info,
                                     "ModbusService",
                                     "read_ok_cwt%u_s%u",
                                     static_cast<unsigned int>(localEntry.deviceIndex),
                                     static_cast<unsigned int>(readConfig.slaveId));
      }
    }
    recordPollSuccess(localEntry.role);
    {
      mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
      if (i < pollEntryCount) {
        pollEntries[i].lastPollMs = nowMs;
      }
      pollCursor = static_cast<uint8_t>((i + 1U) % count);
    }
    touchHeartbeat();
    rtos::ThisThread::sleep_for(std::chrono::milliseconds(SystemConfig::kModbusInterRequestDelayMs));
    return;
  }
}
