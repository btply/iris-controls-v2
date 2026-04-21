#line 1 "/home/billy/Documents/Code/iris-controls-v2/src/hal/IoHalLite.cpp"
#include "IoHalLite.h"

#include "../config/CurtainHardware.h"
#include "../services/LoggerService.h"
#include <Arduino.h>
#include <OptaBlue.h>
#include <mbed.h>

namespace {

rtos::Mutex s_ioMutex;

bool s_initialized = false;
bool s_directionHigh = false;
bool s_outputEnabled[CurtainHardware::kOutputChannelCount] = {false, false, false, false};
bool s_expansionBound = false;
bool s_expansionConfigured = false;
int8_t s_boundExpansionIndex = -1;
uint8_t s_expansionOutputState = 0U;
bool s_expansionWriteHealthy = true;
bool s_expansionOutputStateUncertain = false;
unsigned long s_expansionWriteFailures = 0UL;
unsigned long s_expansionConsecutiveFailures = 0UL;
unsigned long s_lastRebindAttemptMs = 0UL;
const unsigned long kRebindIntervalMs = 5000UL;

bool applyOnboardOutput(uint8_t channel, bool enabled) {
  const int level = enabled ? HIGH : LOW;
  if (channel == 0U) {
    digitalWrite(CurtainHardware::kOutput1Pin, level);
    digitalWrite(CurtainHardware::kOutput1LedPin, level);
    return true;
  }
  if (channel == 1U) {
    digitalWrite(CurtainHardware::kOutput2Pin, level);
    digitalWrite(CurtainHardware::kOutput2LedPin, level);
    return true;
  }
  if (channel == 2U) {
    digitalWrite(CurtainHardware::kOutput3Pin, level);
    digitalWrite(CurtainHardware::kOutput3LedPin, level);
    return true;
  }
  return false;
}

bool bindExpansionLocked() {
  s_boundExpansionIndex = -1;
  const int expansionCount = OptaController.getExpansionNum();
  for (int i = 0; i < expansionCount; i++) {
    const int expansionType = OptaController.getExpansionType(i);
    if (expansionType == EXPANSION_OPTA_DIGITAL_MEC ||
        expansionType == EXPANSION_OPTA_DIGITAL_STS) {
      s_boundExpansionIndex = static_cast<int8_t>(i);
      s_expansionBound = true;
      s_expansionConfigured = true;
      return true;
    }
  }
  s_expansionBound = false;
  s_expansionConfigured = false;
  return false;
}

bool writeExpansionChannelLocked(uint8_t channel, bool enabled) {
  if (s_boundExpansionIndex < 0 || !s_expansionBound || !s_expansionConfigured) {
    s_expansionWriteHealthy = false;
    s_expansionOutputStateUncertain = true;
    s_expansionWriteFailures++;
    s_expansionConsecutiveFailures++;
    return false;
  }

  Opta::Expansion* expansion = OptaController.getExpansionPtr(s_boundExpansionIndex);
  if (expansion == nullptr) {
    s_expansionWriteHealthy = false;
    s_expansionOutputStateUncertain = true;
    s_expansionWriteFailures++;
    s_expansionConsecutiveFailures++;
    return false;
  }

  if (enabled) {
    s_expansionOutputState |= static_cast<uint8_t>(1U << channel);
  } else {
    s_expansionOutputState &= static_cast<uint8_t>(~(1U << channel));
  }

  expansion->write(ADD_DIGITAL_OUTPUT, static_cast<unsigned int>(s_expansionOutputState));
  if (expansion->execute(SET_DIGITAL_OUTPUT) == 0U) {
    s_expansionWriteHealthy = true;
    s_expansionOutputStateUncertain = false;
    s_expansionConsecutiveFailures = 0UL;
    return true;
  }

  s_expansionWriteHealthy = false;
  s_expansionOutputStateUncertain = true;
  s_expansionWriteFailures++;
  s_expansionConsecutiveFailures++;
  return false;
}

bool anyOutputEnabledLocked() {
  for (uint8_t i = 0; i < CurtainHardware::kOutputChannelCount; i++) {
    if (s_outputEnabled[i]) {
      return true;
    }
  }
  return false;
}

bool writeOutputLocked(uint8_t outputChannel, bool enabled, bool warnIfUnavailable) {
  if (outputChannel >= CurtainHardware::kOutputChannelCount) {
    return false;
  }

  bool ok = false;
  if (outputChannel <= 2U) {
    ok = applyOnboardOutput(outputChannel, enabled);
  } else {
    ok = writeExpansionChannelLocked(CurtainHardware::kOutput4ExpansionChannel, enabled);
  }

  if (ok) {
    s_outputEnabled[outputChannel] = enabled;
    return true;
  }
  if (warnIfUnavailable) {
    LoggerService::warn("IoHal", "output_unavailable");
  }
  return false;
}

void disableAllOutputsLocked() {
  for (uint8_t i = 0; i < CurtainHardware::kOutputChannelCount; i++) {
    (void)writeOutputLocked(i, false, false);
  }
}

}  // namespace

namespace IoHal {

void begin() {
  mbed::ScopedLock<rtos::Mutex> lock(s_ioMutex);

  pinMode(CurtainHardware::kDirectionPin, OUTPUT);
  pinMode(CurtainHardware::kDirectionLedPin, OUTPUT);
  pinMode(CurtainHardware::kOutput1Pin, OUTPUT);
  pinMode(CurtainHardware::kOutput1LedPin, OUTPUT);
  pinMode(CurtainHardware::kOutput2Pin, OUTPUT);
  pinMode(CurtainHardware::kOutput2LedPin, OUTPUT);
  pinMode(CurtainHardware::kOutput3Pin, OUTPUT);
  pinMode(CurtainHardware::kOutput3LedPin, OUTPUT);

  s_directionHigh = false;
  const int level = LOW;
  digitalWrite(CurtainHardware::kDirectionPin, level);
  digitalWrite(CurtainHardware::kDirectionLedPin, level);

  disableAllOutputsLocked();
  bindExpansionLocked();

  s_initialized = true;
  LoggerService::info("IoHal", "initialized");
}

void update() {
  mbed::ScopedLock<rtos::Mutex> lock(s_ioMutex);
  const unsigned long nowMs = millis();
  if (!s_expansionBound && nowMs - s_lastRebindAttemptMs >= kRebindIntervalMs) {
    s_lastRebindAttemptMs = nowMs;
    if (bindExpansionLocked()) {
      LoggerService::info("IoHal", "expansion_bound");
    }
  }
}

bool writeSharedSignalLevel(bool high, bool enforceOutputInterlock) {
  mbed::ScopedLock<rtos::Mutex> lock(s_ioMutex);
  if (enforceOutputInterlock && high != s_directionHigh && anyOutputEnabledLocked()) {
    disableAllOutputsLocked();
  }
  s_directionHigh = high;
  const int lvl = high ? HIGH : LOW;
  digitalWrite(CurtainHardware::kDirectionPin, lvl);
  digitalWrite(CurtainHardware::kDirectionLedPin, lvl);
  return true;
}

bool writeOutput(uint8_t outputChannel, bool enabled, bool warnIfUnavailable) {
  mbed::ScopedLock<rtos::Mutex> lock(s_ioMutex);
  return writeOutputLocked(outputChannel, enabled, warnIfUnavailable);
}

void disableAllOutputs() {
  mbed::ScopedLock<rtos::Mutex> lock(s_ioMutex);
  disableAllOutputsLocked();
}

bool isAnyOutputEnabled() {
  mbed::ScopedLock<rtos::Mutex> lock(s_ioMutex);
  return anyOutputEnabledLocked();
}

bool isOutputAvailable(uint8_t outputChannel) {
  mbed::ScopedLock<rtos::Mutex> lock(s_ioMutex);
  if (outputChannel <= 2U) {
    return true;
  }
  return s_expansionBound && s_expansionConfigured;
}

Status getStatus() {
  mbed::ScopedLock<rtos::Mutex> lock(s_ioMutex);
  Status status = {};
  status.initialized = s_initialized;
  status.expansionBound = s_expansionBound;
  status.expansionConfigured = s_expansionConfigured;
  status.expansionWriteHealthy = s_expansionWriteHealthy;
  status.expansionOutputStateUncertain = s_expansionOutputStateUncertain;
  status.expansionWriteFailures = s_expansionWriteFailures;
  status.expansionConsecutiveFailures = s_expansionConsecutiveFailures;
  return status;
}

}  // namespace IoHal
