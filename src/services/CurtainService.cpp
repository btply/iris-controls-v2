#include "CurtainService.h"

#include "../config/SystemConfig.h"
#include "../hal/IoHalLite.h"
#include "LoggerService.h"
#include <Arduino.h>
#include <chrono>
#include <mbed.h>

void CurtainService::begin() {
  for (uint8_t i = 0; i < AppDataConfig::kCurtainCount; i++) {
    controllers[i].begin();
  }

  {
    mbed::ScopedLock<rtos::Mutex> lock(commandMutex);
    desiredTargets = {0.0f, 0.0f, 0.0f, 0.0f};
    desiredEmergencyClose = false;
  }
  {
    mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
    currentPlan = CurtainPlan();
    currentPlan.valid = true;
    lastLoopMs = 0UL;
  }
}

void CurtainService::start() {
  if (running.load()) {
    return;
  }
  running.store(true);
  const osStatus st = workerThread.start(mbed::callback(this, &CurtainService::runThread));
  if (st != osOK) {
    running.store(false);
    LoggerService::enqueue(LoggerService::Level::Error, "CurtainService", "thread_start_failed");
  }
}

void CurtainService::stop() {
  if (!running.load()) {
    return;
  }
  running.store(false);
  workerThread.join();
}

void CurtainService::setTargets(
    const std::array<float, AppDataConfig::kCurtainCount>& targets,
    bool emergencyClose) {
  {
    mbed::ScopedLock<rtos::Mutex> lock(commandMutex);
    desiredTargets = targets;
    desiredEmergencyClose = emergencyClose;
  }
}

CurtainPlan CurtainService::getPlanSnapshot() const {
  mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
  return currentPlan;
}

CurtainService::Health CurtainService::getHealth() const {
  Health health;
  mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
  health.running = running.load();
  health.lastLoopMs = lastLoopMs;
  return health;
}

void CurtainService::runThread() {
  const float deadband = SystemConfig::kCurtainPositionDeadband;
  const float emergencyCloseEpsilon = SystemConfig::kCurtainEmergencyCloseEpsilon;
  const unsigned long openSliceMs = SystemConfig::kCurtainOpenSliceMs;
  const unsigned long closeSliceMs = SystemConfig::kCurtainCloseSliceMs;

  while (running.load()) {
    const unsigned long nowMs = millis();

    std::array<float, AppDataConfig::kCurtainCount> localTargets;
    bool localEmergencyClose = false;
    {
      mbed::ScopedLock<rtos::Mutex> lock(commandMutex);
      localTargets = desiredTargets;
      localEmergencyClose = desiredEmergencyClose;
    }

    for (uint8_t i = 0; i < AppDataConfig::kCurtainCount; i++) {
      if (localEmergencyClose) {
        controllers[i].emergencyClose();
      } else {
        controllers[i].setTarget(localTargets[i]);
      }
    }

    std::array<bool, AppDataConfig::kCurtainCount> wantOpen = {false, false, false, false};
    std::array<bool, AppDataConfig::kCurtainCount> wantClose = {false, false, false, false};
    std::array<bool, AppDataConfig::kCurtainCount> ranOpen = {false, false, false, false};
    std::array<bool, AppDataConfig::kCurtainCount> ranClose = {false, false, false, false};

    bool anyNeedOpen = false;
    bool anyNeedClose = false;
    for (uint8_t i = 0; i < AppDataConfig::kCurtainCount; i++) {
      if (localEmergencyClose) {
        wantClose[i] = controllers[i].currentPosition() > emergencyCloseEpsilon;
        anyNeedClose = anyNeedClose || wantClose[i];
        continue;
      }
      wantOpen[i] = controllers[i].needOpen(deadband);
      wantClose[i] = controllers[i].needClose(deadband);
      anyNeedOpen = anyNeedOpen || wantOpen[i];
      anyNeedClose = anyNeedClose || wantClose[i];
    }

    if (anyNeedOpen) {
      for (uint8_t i = 0; i < AppDataConfig::kCurtainCount; i++) {
        if (wantOpen[i] && controllers[i].applyOpen()) {
          ranOpen[i] = true;
        }
      }
      rtos::ThisThread::sleep_for(std::chrono::milliseconds(openSliceMs));
      for (uint8_t i = 0; i < AppDataConfig::kCurtainCount; i++) {
        if (ranOpen[i]) {
          controllers[i].stop();
        }
      }
      if (!IoHal::isAnyOutputEnabled()) {
        IoHal::writeSharedSignalLevel(false, false);
      }
      for (uint8_t i = 0; i < AppDataConfig::kCurtainCount; i++) {
        if (ranOpen[i]) {
          controllers[i].advancePosition(true, openSliceMs);
        }
      }
    }

    if (anyNeedClose) {
      for (uint8_t i = 0; i < AppDataConfig::kCurtainCount; i++) {
        if (wantClose[i] && controllers[i].applyClose()) {
          ranClose[i] = true;
        }
      }
      rtos::ThisThread::sleep_for(std::chrono::milliseconds(closeSliceMs));
      for (uint8_t i = 0; i < AppDataConfig::kCurtainCount; i++) {
        if (ranClose[i]) {
          controllers[i].stop();
        }
      }
      if (!IoHal::isAnyOutputEnabled()) {
        IoHal::writeSharedSignalLevel(false, false);
      }
      for (uint8_t i = 0; i < AppDataConfig::kCurtainCount; i++) {
        if (ranClose[i]) {
          controllers[i].advancePosition(false, closeSliceMs);
        }
      }
    }

    CurtainPlan localPlan;
    localPlan.valid = true;
    localPlan.emergencyClose = localEmergencyClose;
    for (uint8_t i = 0; i < AppDataConfig::kCurtainCount; i++) {
      localPlan.targetByCurtain[i] = controllers[i].targetPosition();
    }

    {
      mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
      currentPlan = localPlan;
      lastLoopMs = nowMs;
    }

    const unsigned long elapsed =
        (anyNeedOpen ? openSliceMs : 0UL) + (anyNeedClose ? closeSliceMs : 0UL);
    if (SystemConfig::kCurtainLoopSleepMs > elapsed) {
      rtos::ThisThread::sleep_for(
          std::chrono::milliseconds(SystemConfig::kCurtainLoopSleepMs - elapsed));
    }
  }

  for (uint8_t i = 0; i < AppDataConfig::kCurtainCount; i++) {
    controllers[i].stop();
  }
  LoggerService::info("CurtainService", "stopped");
}
