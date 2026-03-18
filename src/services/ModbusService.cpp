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

  weatherDevice.reset();
  for (uint8_t i = 0; i < kCwtSensorCount; i++) {
    cwtDevices[i].reset();
  }

  stateMutex.unlock();

  RS485.setDelays(200, 1500);

  ModbusRTUClient.begin(9600, SERIAL_8N1);
  ModbusRTUClient.setTimeout(200);

  workerThread.start(mbed::callback(this, &ModbusService::runThread));
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

ModbusService::WeatherLatestSnapshot ModbusService::getLatestWeather() const {
  WeatherLatestSnapshot latest;
  const unsigned long nowMs = millis();
  stateMutex.lock();
  latest.data = weatherDevice.getSnapshot();
  latest.stale = !latest.data.valid || latest.data.lastUpdateMs == 0UL ||
                 nowMs < latest.data.lastUpdateMs ||
                 (nowMs - latest.data.lastUpdateMs) > SystemConfig::kWeatherFreshMaxAgeMs;
  stateMutex.unlock();
  return latest;
}

ModbusService::CwtLatestSnapshot ModbusService::getLatestCwt(uint8_t index) const {
  CwtLatestSnapshot latest;
  if (index >= kCwtSensorCount) {
    return latest;
  }

  const unsigned long nowMs = millis();
  stateMutex.lock();
  latest.data = cwtDevices[index].getSnapshot();
  latest.stale = !latest.data.valid || latest.data.lastUpdateMs == 0UL ||
                 nowMs < latest.data.lastUpdateMs ||
                 (nowMs - latest.data.lastUpdateMs) > SystemConfig::kCwtFreshMaxAgeMs;
  stateMutex.unlock();
  return latest;
}

ModbusService::ModbusSnapshot ModbusService::getLatestSnapshot() const {
  ModbusSnapshot latest;
  const unsigned long nowMs = millis();
  stateMutex.lock();
  latest.weather.data = weatherDevice.getSnapshot();
  latest.weather.stale = !latest.weather.data.valid || latest.weather.data.lastUpdateMs == 0UL ||
                         nowMs < latest.weather.data.lastUpdateMs ||
                         (nowMs - latest.weather.data.lastUpdateMs) >
                             SystemConfig::kWeatherFreshMaxAgeMs;

  for (uint8_t i = 0; i < kCwtSensorCount; i++) {
    latest.cwt[i].data = cwtDevices[i].getSnapshot();
    latest.cwt[i].stale = !latest.cwt[i].data.valid || latest.cwt[i].data.lastUpdateMs == 0UL ||
                          nowMs < latest.cwt[i].data.lastUpdateMs ||
                          (nowMs - latest.cwt[i].data.lastUpdateMs) >
                              SystemConfig::kCwtFreshMaxAgeMs;
  }
  stateMutex.unlock();
  return latest;
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

void ModbusService::touchHeartbeat() {
  stateMutex.lock();
  lastLoopMs = millis();
  stateMutex.unlock();
}

void ModbusService::runThread() {
  while (running.load()) {
    touchHeartbeat();
    const unsigned long nowMs = millis();
    touchHeartbeat();
    readWeatherIfDue(nowMs);
    touchHeartbeat();
    readCwtIfDue(nowMs);
    touchHeartbeat();
    rtos::ThisThread::sleep_for(SystemConfig::kModbusLoopSleepMs);
  }
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
    weatherDevice.markInvalid();
    stateMutex.unlock();
    if (shouldInvalidate) {
      sharedState->invalidateWeather();
    }
    return;
  }

  stateMutex.lock();
  weatherDevice.updateFromSnapshot(weather);
  stateMutex.unlock();
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
      stateMutex.lock();
      cwtDevices[i].markInvalid();
      stateMutex.unlock();
      sharedState->invalidateCwt(i);
      continue;
    }
    stateMutex.lock();
    cwtDevices[i].updateFromSnapshot(cwt);
    stateMutex.unlock();
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
  if (shouldInvalidateAll) {
    for (uint8_t i = 0; i < kCwtSensorCount; i++) {
      cwtDevices[i].markInvalid();
    }
  }
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

  stateMutex.lock();
  const bool decoded = weatherDevice.updateFromRegisters(regs, 8U, nowMs);
  if (decoded) {
    *out = weatherDevice.getSnapshot();
  }
  stateMutex.unlock();
  if (!decoded) {
    return false;
  }
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

  stateMutex.lock();
  const bool decoded = cwtDevices[sensorIndex].updateFromRegisters(regs, 2U, nowMs);
  if (decoded) {
    *out = cwtDevices[sensorIndex].getSnapshot();
  }
  stateMutex.unlock();
  if (!decoded) {
    return false;
  }
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
