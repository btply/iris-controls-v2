#include "CurtainService.h"

#include "../config/SystemConfig.h"
#include "../hal/IoHalLite.h"
#include "LoggerService.h"
#include <Arduino.h>
#include <chrono>

void CurtainService::onCurtainTimeout(uint8_t index) {
  if (index >= AppDataConfig::kCurtainCount) {
    return;
  }
  const uint8_t ch = controllers[index].getOutputChannel();
  // Accepted tradeoff for now: direct best-effort cutoff in timeout callback.
  // This keeps stop latency tight before first flash across current hardware setup.
  (void)IoHal::writeOutput(ch, false, false);
}

void CurtainService::onCurtainTimeout0() { onCurtainTimeout(0); }
void CurtainService::onCurtainTimeout1() { onCurtainTimeout(1); }
void CurtainService::onCurtainTimeout2() { onCurtainTimeout(2); }
void CurtainService::onCurtainTimeout3() { onCurtainTimeout(3); }

void CurtainService::armCurtainTimeout(uint8_t index, unsigned long runMs) {
  if (index >= AppDataConfig::kCurtainCount || runMs == 0UL) {
    return;
  }
  movementTimeouts[index].detach();
  switch (index) {
    case 0:
      movementTimeouts[0].attach(
          mbed::callback(this, &CurtainService::onCurtainTimeout0),
          std::chrono::milliseconds(runMs));
      break;
    case 1:
      movementTimeouts[1].attach(
          mbed::callback(this, &CurtainService::onCurtainTimeout1),
          std::chrono::milliseconds(runMs));
      break;
    case 2:
      movementTimeouts[2].attach(
          mbed::callback(this, &CurtainService::onCurtainTimeout2),
          std::chrono::milliseconds(runMs));
      break;
    case 3:
      movementTimeouts[3].attach(
          mbed::callback(this, &CurtainService::onCurtainTimeout3),
          std::chrono::milliseconds(runMs));
      break;
    default:
      break;
  }
}

void CurtainService::disarmAllCurtainTimeouts() {
  for (uint8_t i = 0; i < AppDataConfig::kCurtainCount; i++) {
    movementTimeouts[i].detach();
  }
}

void CurtainService::begin() {
  disarmAllCurtainTimeouts();
  for (uint8_t i = 0; i < AppDataConfig::kCurtainCount; i++) {
    controllers[i].begin();
  }

  commandMutex.lock();
  desiredTargets = {0.0f, 0.0f, 0.0f, 0.0f};
  desiredEmergencyClose = false;
  commandMutex.unlock();

  stateMutex.lock();
  currentPlan = CurtainPlan();
  currentPlan.valid = true;
  lastLoopMs = 0UL;
  stateMutex.unlock();
}

void CurtainService::start() {
  if (running.load()) {
    return;
  }
  running.store(true);
  workerThread.start(mbed::callback(this, &CurtainService::runThread));
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
  commandMutex.lock();
  desiredTargets = targets;
  desiredEmergencyClose = emergencyClose;
  commandMutex.unlock();
}

CurtainPlan CurtainService::getPlanSnapshot() const {
  CurtainPlan plan;
  stateMutex.lock();
  plan = currentPlan;
  stateMutex.unlock();
  return plan;
}

CurtainService::Health CurtainService::getHealth() const {
  Health health;
  stateMutex.lock();
  health.running = running.load();
  health.lastLoopMs = lastLoopMs;
  stateMutex.unlock();
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
    commandMutex.lock();
    localTargets = desiredTargets;
    localEmergencyClose = desiredEmergencyClose;
    commandMutex.unlock();

    for (uint8_t i = 0; i < AppDataConfig::kCurtainCount; i++) {
      if (localEmergencyClose) {
        controllers[i].emergencyClose();
      } else {
        controllers[i].setTarget(localTargets[i]);
      }
    }

    std::array<bool, AppDataConfig::kCurtainCount> wantOpen = {false, false, false,
                                                                    false};
    std::array<bool, AppDataConfig::kCurtainCount> wantClose = {false, false, false,
                                                                     false};
    std::array<bool, AppDataConfig::kCurtainCount> ranOpen = {false, false, false,
                                                                   false};
    std::array<bool, AppDataConfig::kCurtainCount> ranClose = {false, false, false,
                                                                    false};

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
          armCurtainTimeout(i, openSliceMs);
        }
      }
      rtos::ThisThread::sleep_for(openSliceMs);
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
          armCurtainTimeout(i, closeSliceMs);
        }
      }
      rtos::ThisThread::sleep_for(closeSliceMs);
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

    stateMutex.lock();
    currentPlan = localPlan;
    lastLoopMs = nowMs;
    stateMutex.unlock();

    const unsigned long elapsed =
        (anyNeedOpen ? openSliceMs : 0UL) + (anyNeedClose ? closeSliceMs : 0UL);
    if (SystemConfig::kCurtainLoopSleepMs > elapsed) {
      rtos::ThisThread::sleep_for(SystemConfig::kCurtainLoopSleepMs - elapsed);
    }
  }

  disarmAllCurtainTimeouts();
  for (uint8_t i = 0; i < AppDataConfig::kCurtainCount; i++) {
    controllers[i].stop();
  }
  LoggerService::info("CurtainService", "stopped");
}
