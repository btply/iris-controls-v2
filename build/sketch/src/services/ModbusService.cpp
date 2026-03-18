#line 1 "/home/billy/Documents/Code/iris-controls-v2/src/services/ModbusService.cpp"
#include "ModbusService.h"

#include "../config/SystemConfig.h"
#include "LoggerService.h"
#include <Arduino.h>
#include <ArduinoModbus.h>
#include <ArduinoRS485.h>

const uint8_t ModbusService::kCwtSlaveIds[ModbusService::kCwtSensorCount] = {2U, 3U,
                                                                              4U, 5U};

void ModbusService::begin(SharedState* sharedStateIn) {
  sharedState = sharedStateIn;
  stateMutex.lock();
  busReady = false;
  busReadyLogged = false;
  lastBusInitAttemptMs = 0UL;
  lastLoopMs = 0UL;
  lastWeatherSuccessMs = 0UL;
  lastCwtSuccessMs = 0UL;
  totalReadFailures = 0UL;
  weatherConsecutiveFailures = 0UL;
  cwtConsecutiveFailures = 0UL;
  lastWeatherPollMs = 0UL;
  lastCwtPollMs = 0UL;
  stateMutex.unlock();
}

void ModbusService::start() {
  if (running.load() || sharedState == nullptr) {
    return;
  }
  running.store(true);
  workerThread.start(mbed::callback(this, &ModbusService::runThread));
}

void ModbusService::stop() {
  if (!running.load()) {
    return;
  }
  running.store(false);
  workerThread.join();
}

bool ModbusService::hasFreshData(unsigned long nowMs) const {
  stateMutex.lock();
  const bool fresh = isWeatherFreshLocked(nowMs) && isCwtFreshLocked(nowMs) &&
                     !isDegradedLocked();
  stateMutex.unlock();
  return fresh;
}

ModbusService::Health ModbusService::getHealth(unsigned long nowMs) const {
  Health health;
  stateMutex.lock();
  health.busReady = busReady;
  health.lastLoopMs = lastLoopMs;
  health.lastWeatherSuccessMs = lastWeatherSuccessMs;
  health.lastCwtSuccessMs = lastCwtSuccessMs;
  health.totalReadFailures = totalReadFailures;
  health.weatherConsecutiveFailures = weatherConsecutiveFailures;
  health.cwtConsecutiveFailures = cwtConsecutiveFailures;
  health.weatherFresh = isWeatherFreshLocked(nowMs);
  health.cwtFresh = isCwtFreshLocked(nowMs);
  health.degraded = isDegradedLocked();
  stateMutex.unlock();
  return health;
}

bool ModbusService::isWeatherFreshLocked(unsigned long nowMs) const {
  if (lastWeatherSuccessMs == 0UL || nowMs < lastWeatherSuccessMs) {
    return false;
  }
  return (nowMs - lastWeatherSuccessMs) <= SystemConfig::kWeatherFreshMaxAgeMs;
}

bool ModbusService::isCwtFreshLocked(unsigned long nowMs) const {
  if (lastCwtSuccessMs == 0UL || nowMs < lastCwtSuccessMs) {
    return false;
  }
  return (nowMs - lastCwtSuccessMs) <= SystemConfig::kCwtFreshMaxAgeMs;
}

bool ModbusService::isDegradedLocked() const {
  return weatherConsecutiveFailures >= SystemConfig::kModbusFailureDegradeThreshold ||
         cwtConsecutiveFailures >= SystemConfig::kModbusFailureDegradeThreshold;
}

void ModbusService::runThread() {
  while (running.load()) {
    const unsigned long nowMs = millis();
    stateMutex.lock();
    lastLoopMs = nowMs;
    stateMutex.unlock();
    if (!ensureBusReady(nowMs)) {
      rtos::ThisThread::sleep_for(SystemConfig::kModbusLoopSleepMs);
      continue;
    }
    readWeatherIfDue(nowMs);
    readCwtIfDue(nowMs);
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

#if defined(ARDUINO_ARCH_MBED)
  RS485.begin(9600, SERIAL_8N1);
#else
  if (!RS485.begin(9600, SERIAL_8N1)) {
    LoggerService::warn("ModbusService", "rs485_begin_failed");
    return false;
  }
#endif
  if (!ModbusRTUClient.begin(9600, SERIAL_8N1)) {
    LoggerService::warn("ModbusService", "modbus_begin_failed");
    return false;
  }
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
  if (!ModbusRTUClient.requestFrom(slaveId, HOLDING_REGISTERS, startRegister,
                                   registerCount)) {
    return false;
  }
  for (uint16_t i = 0; i < registerCount; i++) {
    if (!ModbusRTUClient.available()) {
      return false;
    }
    outRegisters[i] = static_cast<uint16_t>(ModbusRTUClient.read());
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
  if (!ModbusRTUClient.requestFrom(slaveId, INPUT_REGISTERS, startRegister,
                                   registerCount)) {
    return false;
  }
  for (uint16_t i = 0; i < registerCount; i++) {
    if (!ModbusRTUClient.available()) {
      return false;
    }
    outRegisters[i] = static_cast<uint16_t>(ModbusRTUClient.read());
  }
  return true;
}

void ModbusService::readWeatherIfDue(unsigned long nowMs) {
  if (nowMs - lastWeatherPollMs < SystemConfig::kWeatherPollIntervalMs) {
    return;
  }
  lastWeatherPollMs = nowMs;

  WeatherSnapshot weather;
  bool ok = readWeatherFromBus(&weather, nowMs);
  if (!ok && kUseSyntheticDataIfBusUnavailable) {
    fillSyntheticWeather(&weather, nowMs);
    ok = true;
  }
  if (!ok) {
    stateMutex.lock();
    totalReadFailures++;
    weatherConsecutiveFailures++;
    const bool shouldInvalidate =
        weatherConsecutiveFailures >= SystemConfig::kModbusFailureDegradeThreshold;
    stateMutex.unlock();
    if (shouldInvalidate) {
      sharedState->invalidateWeather();
    }
    return;
  }

  sharedState->updateWeather(weather);
  stateMutex.lock();
  lastWeatherSuccessMs = nowMs;
  weatherConsecutiveFailures = 0UL;
  stateMutex.unlock();
}

void ModbusService::readCwtIfDue(unsigned long nowMs) {
  if (nowMs - lastCwtPollMs < SystemConfig::kCwtPollIntervalMs) {
    return;
  }
  lastCwtPollMs = nowMs;

  bool readAny = false;
  bool failedAny = false;
  for (uint8_t i = 0; i < kCwtSensorCount; i++) {
    CwtSnapshot cwt;
    bool ok = readCwtFromBus(i, &cwt, nowMs);
    if (!ok && kUseSyntheticDataIfBusUnavailable) {
      fillSyntheticCwt(i, &cwt, nowMs);
      ok = true;
    }
    if (!ok) {
      failedAny = true;
      sharedState->invalidateCwt(i);
      continue;
    }
    sharedState->updateCwt(i, cwt);
    readAny = true;
  }

  stateMutex.lock();
  if (readAny) {
    lastCwtSuccessMs = nowMs;
    cwtConsecutiveFailures = 0UL;
  } else if (failedAny) {
    totalReadFailures++;
    cwtConsecutiveFailures++;
  }
  const bool shouldInvalidateAll =
      cwtConsecutiveFailures >= SystemConfig::kModbusFailureDegradeThreshold;
  stateMutex.unlock();
  if (shouldInvalidateAll) {
    sharedState->invalidateAllCwt();
  }
}

bool ModbusService::readWeatherFromBus(WeatherSnapshot* out,
                                       unsigned long nowMs) {
  if (out == nullptr) {
    return false;
  }

  // Weather station values are int32 over two words, scaled by /1000.
  // Read from 0x0012 to 0x0019 (8 words) to get:
  // - wind speed avg at 0x0012/0x0013
  // - rain intensity at 0x0018/0x0019
  uint16_t regs[8] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
  if (!readInputRegisters(kWeatherSlaveId, 0x0012U, 8U, regs)) {
    return false;
  }

  const int32_t windRaw =
      (static_cast<int32_t>(regs[0]) << 16) | static_cast<int32_t>(regs[1]);
  const int32_t rainRaw =
      (static_cast<int32_t>(regs[6]) << 16) | static_cast<int32_t>(regs[7]);

  out->valid = true;
  out->windSpeedMps = static_cast<float>(windRaw) / 1000.0f;
  out->rainDetected = (static_cast<float>(rainRaw) / 1000.0f) > 0.0f;
  out->lastUpdateMs = nowMs;
  return true;
}

bool ModbusService::readCwtFromBus(uint8_t sensorIndex,
                                   CwtSnapshot* out,
                                   unsigned long nowMs) {
  if (sensorIndex >= kCwtSensorCount) {
    return false;
  }
  if (out == nullptr) {
    return false;
  }

  // CWT map:
  // - 0x0000 humidity (0.1 %RH)
  // - 0x0001 temperature (signed 0.1 C)
  uint16_t regs[2] = {0U, 0U};
  if (!readHoldingRegisters(kCwtSlaveIds[sensorIndex], 0x0000U, 2U, regs)) {
    return false;
  }

  out->valid = true;
  out->rhPct = static_cast<float>(regs[0]) * 0.1f;
  out->tempC = static_cast<float>(static_cast<int16_t>(regs[1])) * 0.1f;
  out->lastUpdateMs = nowMs;
  return true;
}

void ModbusService::fillSyntheticWeather(WeatherSnapshot* out,
                                         unsigned long nowMs) const {
  if (out == nullptr) {
    return;
  }
  out->valid = true;
  out->rainDetected = false;
  out->windSpeedMps = 3.0f + static_cast<float>((nowMs / 1000UL) % 4U);
  out->lastUpdateMs = nowMs;
}

void ModbusService::fillSyntheticCwt(uint8_t sensorIndex,
                                     CwtSnapshot* out,
                                     unsigned long nowMs) const {
  if (out == nullptr) {
    return;
  }
  const float phase = static_cast<float>((nowMs / 1000UL + sensorIndex) % 10U);
  out->valid = true;
  out->tempC = 22.0f + (phase * 0.1f);
  out->rhPct = 74.0f + (phase * 0.3f);
  out->lastUpdateMs = nowMs;
}
