#include "ModbusService.h"

#include "../config/SystemConfig.h"
#include "LoggerService.h"
#include <Arduino.h>
#include <ArduinoModbus.h>
#include <ArduinoRS485.h>

void ModbusService::begin() {
  stateMutex.lock();
  pollEntryCount = 0U;
  pollCursor = 0U;
  busReady = false;
  busReadyLogged = false;
  lastBusInitAttemptMs = 0UL;
  lastLoopMs.store(0UL);
  totalReadFailures = 0UL;
  weatherConsecutiveFailures = 0UL;
  cwtConsecutiveFailures = 0UL;
  for (uint8_t i = 0; i < kMaxDevices; i++) {
    pollEntries[i] = PollEntry{};
  }
  stateMutex.unlock();
}

bool ModbusService::registerDevice(IModbusDevice& device,
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

  stateMutex.lock();
  if (pollEntryCount >= kMaxDevices) {
    stateMutex.unlock();
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
  pollEntryCount++;
  stateMutex.unlock();
  return true;
}

void ModbusService::start() {
  if (running.load()) {
    return;
  }
  running.store(true);
  const osStatus status = workerThread.start(mbed::callback(this, &ModbusService::runThread));
  if (status != osOK) {
    running.store(false);
    LoggerService::enqueue(LoggerService::Level::Error, "ModbusService", "thread_start_failed");
  }
}

void ModbusService::stop() {
  if (!running.load()) {
    return;
  }
  running.store(false);
  workerThread.join();
}

ModbusService::Health ModbusService::getHealth() const {
  Health health;
  stateMutex.lock();
  health.busReady = busReady;
  health.lastLoopMs = lastLoopMs.load();
  health.totalReadFailures = totalReadFailures;
  health.weatherConsecutiveFailures = weatherConsecutiveFailures;
  health.cwtConsecutiveFailures = cwtConsecutiveFailures;
  health.degraded = isDegradedLocked();
  stateMutex.unlock();
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
  while (running.load()) {
    touchHeartbeat();
    const unsigned long nowMs = millis();
    if (!ensureBusReady(nowMs)) {
      touchHeartbeat();
      rtos::ThisThread::sleep_for(SystemConfig::kModbusLoopSleepMs);
      continue;
    }
    touchHeartbeat();
    pollDevices(nowMs);
    touchHeartbeat();
    rtos::ThisThread::sleep_for(SystemConfig::kModbusLoopSleepMs);
  }
}

bool ModbusService::ensureBusReady(unsigned long nowMs) {
  {
    stateMutex.lock();
    if (busReady) {
      stateMutex.unlock();
      return true;
    }
    if (nowMs - lastBusInitAttemptMs < 2000UL) {
      stateMutex.unlock();
      return false;
    }
    lastBusInitAttemptMs = nowMs;
    stateMutex.unlock();
  }

  touchHeartbeat();

  RS485.setDelays(200, 200);

  touchHeartbeat();
  if (!ModbusRTUClient.begin(9600, SERIAL_8N1)) {
    LoggerService::enqueue(LoggerService::Level::Warn, "ModbusService", "modbus_begin_failed");
    return false;
  }
  touchHeartbeat();
  ModbusRTUClient.setTimeout(SystemConfig::kModbusResponseTimeoutMs);

  bool shouldLogReady = false;
  stateMutex.lock();
  busReady = true;
  if (!busReadyLogged) {
    busReadyLogged = true;
    shouldLogReady = true;
  }
  stateMutex.unlock();
  if (shouldLogReady) {
    LoggerService::enqueue(LoggerService::Level::Info, "ModbusService", "bus_ready");
  }
  return true;
}

bool ModbusService::readHoldingRegisters(uint8_t slaveId,
                                         uint16_t startRegister,
                                         uint16_t registerCount,
                                         uint16_t* outRegisters) {
  if (outRegisters == nullptr || registerCount == 0U) {
    return false;
  }
  touchHeartbeat();
  if (!ModbusRTUClient.requestFrom(slaveId, HOLDING_REGISTERS, startRegister,
                                   registerCount)) {
    return false;
  }
  touchHeartbeat();
  for (uint16_t i = 0; i < registerCount; i++) {
    touchHeartbeat();
    if (!ModbusRTUClient.available()) {
      return false;
    }
    outRegisters[i] = static_cast<uint16_t>(ModbusRTUClient.read());
    touchHeartbeat();
  }
  return true;
}

bool ModbusService::readInputRegisters(uint8_t slaveId,
                                       uint16_t startRegister,
                                       uint16_t registerCount,
                                       uint16_t* outRegisters) {
  if (outRegisters == nullptr || registerCount == 0U) {
    return false;
  }
  touchHeartbeat();
  if (!ModbusRTUClient.requestFrom(slaveId, INPUT_REGISTERS, startRegister,
                                   registerCount)) {
    return false;
  }
  touchHeartbeat();
  for (uint16_t i = 0; i < registerCount; i++) {
    touchHeartbeat();
    if (!ModbusRTUClient.available()) {
      return false;
    }
    outRegisters[i] = static_cast<uint16_t>(ModbusRTUClient.read());
    touchHeartbeat();
  }
  return true;
}

bool ModbusService::readRegisters(const ModbusReadConfig& config,
                                  uint16_t* outRegisters) {
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
  if (config.registerKind == ModbusRegisterKind::Input) {
    return readInputRegisters(config.slaveId,
                              config.startRegister,
                              config.registerCount,
                              outRegisters);
  }
  return readHoldingRegisters(config.slaveId,
                              config.startRegister,
                              config.registerCount,
                              outRegisters);
}

void ModbusService::recordPollSuccess(DeviceRole role) {
  stateMutex.lock();
  if (role == DeviceRole::Weather) {
    weatherConsecutiveFailures = 0UL;
  } else {
    cwtConsecutiveFailures = 0UL;
  }
  stateMutex.unlock();
}

void ModbusService::recordPollFailure(DeviceRole role) {
  stateMutex.lock();
  totalReadFailures++;
  if (role == DeviceRole::Weather) {
    weatherConsecutiveFailures++;
  } else {
    cwtConsecutiveFailures++;
  }
  stateMutex.unlock();
}

void ModbusService::pollDevices(unsigned long nowMs) {
  uint8_t count = 0U;
  uint8_t startCursor = 0U;
  stateMutex.lock();
  count = pollEntryCount;
  startCursor = pollCursor;
  stateMutex.unlock();
  if (count == 0U) {
    return;
  }

  for (uint8_t offset = 0; offset < count; offset++) {
    const uint8_t i = static_cast<uint8_t>((startCursor + offset) % count);
    PollEntry localEntry;
    bool due = false;
    stateMutex.lock();
    localEntry = pollEntries[i];
    if (localEntry.device != nullptr &&
        (nowMs - localEntry.lastPollMs) >= localEntry.pollIntervalMs) {
      due = true;
    }
    stateMutex.unlock();
    if (!due) {
      continue;
    }

    const ModbusReadConfig readConfig = localEntry.device->getReadConfig();
    uint16_t regs[kMaxRegistersPerPoll] = {};
    bool ok = readRegisters(readConfig, regs);
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
                                   "read_fail_diag role=%s idx=%u sid=%u tbl=%c reg=0x%04X cnt=%u err=%s",
                                   roleLabel,
                                   static_cast<unsigned int>(localEntry.deviceIndex),
                                   static_cast<unsigned int>(readConfig.slaveId),
                                   tableCode,
                                   static_cast<unsigned int>(readConfig.startRegister),
                                   static_cast<unsigned int>(readConfig.registerCount),
                                   lastError);
      localEntry.device->markInvalid();
      recordPollFailure(localEntry.role);
      stateMutex.lock();
      if (i < pollEntryCount) {
        pollEntries[i].lastPollMs = nowMs;
      }
      pollCursor = static_cast<uint8_t>((i + 1U) % count);
      stateMutex.unlock();
      touchHeartbeat();
      rtos::ThisThread::sleep_for(SystemConfig::kModbusInterRequestDelayMs);
      return;
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
    stateMutex.lock();
    if (i < pollEntryCount) {
      pollEntries[i].lastPollMs = nowMs;
    }
    pollCursor = static_cast<uint8_t>((i + 1U) % count);
    stateMutex.unlock();
    touchHeartbeat();
    rtos::ThisThread::sleep_for(SystemConfig::kModbusInterRequestDelayMs);
    return;
  }
}
