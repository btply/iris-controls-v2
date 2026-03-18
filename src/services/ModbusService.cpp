#include "ModbusService.h"

#include "../config/SystemConfig.h"
#include "LoggerService.h"
#include <Arduino.h>
#include <ArduinoModbus.h>
#include <ArduinoRS485.h>

void ModbusService::begin() {
  stateMutex.lock();
  pollEntryCount = 0U;
  busReady = false;
  busReadyLogged = false;
  lastBusInitAttemptMs = 0UL;
  lastLoopMs = 0UL;
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
  entry.device = &device;
  entry.role = role;
  entry.deviceIndex = deviceIndex;
  entry.pollIntervalMs = pollIntervalMs;
  entry.lastPollMs = 0UL;
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
    LoggerService::error("ModbusService", "thread_start_failed");
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
  health.lastLoopMs = lastLoopMs;
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
  stateMutex.lock();
  lastLoopMs = millis();
  stateMutex.unlock();
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

  RS485.setDelays(200, 1500);

  touchHeartbeat();
  if (!ModbusRTUClient.begin(9600, SERIAL_8N1)) {
    LoggerService::warn("ModbusService", "modbus_begin_failed");
    return false;
  }
  touchHeartbeat();
  ModbusRTUClient.setTimeout(200);

  stateMutex.lock();
  busReady = true;
  if (!busReadyLogged) {
    LoggerService::info("ModbusService", "bus_ready");
    busReadyLogged = true;
  }
  stateMutex.unlock();
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
  stateMutex.lock();
  count = pollEntryCount;
  stateMutex.unlock();

  for (uint8_t i = 0; i < count; i++) {
    PollEntry localEntry;
    stateMutex.lock();
    localEntry = pollEntries[i];
    if (nowMs - localEntry.lastPollMs < localEntry.pollIntervalMs) {
      stateMutex.unlock();
      continue;
    }
    pollEntries[i].lastPollMs = nowMs;
    stateMutex.unlock();

    if (localEntry.device == nullptr) {
      continue;
    }

    const ModbusReadConfig readConfig = localEntry.device->getReadConfig();
    uint16_t regs[kMaxRegistersPerPoll] = {};
    bool ok = readRegisters(readConfig, regs);
    if (ok) {
      ok = localEntry.device->updateFromRegisters(regs, readConfig.registerCount, nowMs);
    }

    if (!ok) {
      if (localEntry.role == DeviceRole::Weather) {
        LoggerService::printf(LoggerService::Level::Warn,
                              "ModbusService",
                              "read_fail_weather_s%u",
                              static_cast<unsigned int>(readConfig.slaveId));
      } else {
        LoggerService::printf(LoggerService::Level::Warn,
                              "ModbusService",
                              "read_fail_cwt%u_s%u",
                              static_cast<unsigned int>(localEntry.deviceIndex),
                              static_cast<unsigned int>(readConfig.slaveId));
      }
      localEntry.device->markInvalid();
      recordPollFailure(localEntry.role);
      continue;
    }

    if (localEntry.role == DeviceRole::Weather) {
      LoggerService::printf(LoggerService::Level::Info,
                            "ModbusService",
                            "read_ok_weather_s%u",
                            static_cast<unsigned int>(readConfig.slaveId));
    } else {
      LoggerService::printf(LoggerService::Level::Info,
                            "ModbusService",
                            "read_ok_cwt%u_s%u",
                            static_cast<unsigned int>(localEntry.deviceIndex),
                            static_cast<unsigned int>(readConfig.slaveId));
    }
    recordPollSuccess(localEntry.role);
    touchHeartbeat();
  }
}
