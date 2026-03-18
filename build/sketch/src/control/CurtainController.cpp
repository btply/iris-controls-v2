#line 1 "/home/billy/Documents/Code/iris-controls-v2/src/control/CurtainController.cpp"
#include "CurtainController.h"

#include "../config/CurtainHardware.h"
#include "../hal/IoHalLite.h"
#include "../services/LoggerService.h"

const float CurtainController::kPositionRate = 0.20f;

CurtainController::CurtainController(uint8_t curtainIdIn) : curtainId(curtainIdIn) {
  outputChannelValid =
      CurtainHardware::mapCurtainIdToOutputChannel(curtainId, &outputChannel);
}

void CurtainController::begin() {
  current = 0.0f;
  target = 0.0f;
  emergencyClosed = false;
  motorEnabled = false;
  lastTickMs = 0UL;
  stopMotor();
}

void CurtainController::tick(unsigned long nowMs) {
  if (lastTickMs == 0UL) {
    lastTickMs = nowMs;
    return;
  }

  const unsigned long deltaMs = nowMs - lastTickMs;
  lastTickMs = nowMs;
  const float maxStep = (static_cast<float>(deltaMs) / 1000.0f) * 0.20f;

  if (current < target) {
    if (setMotorOutput(MotorDirection::Opening)) {
      current += maxStep;
      if (current > target) {
        current = target;
      }
    }
  } else if (current > target) {
    if (setMotorOutput(MotorDirection::Closing)) {
      current -= maxStep;
      if (current < target) {
        current = target;
      }
    }
  } else {
    stopMotor();
  }
  current = clamp01(current);
}

void CurtainController::setTarget(float targetPositionIn) {
  if (emergencyClosed && targetPositionIn > 0.0f) {
    emergencyClosed = false;
  }
  target = clamp01(targetPositionIn);
}

void CurtainController::emergencyClose() {
  emergencyClosed = true;
  target = 0.0f;
}

float CurtainController::currentPosition() const {
  return current;
}

float CurtainController::targetPosition() const {
  return target;
}

bool CurtainController::isEmergencyClosed() const {
  return emergencyClosed;
}

bool CurtainController::applyOpen() {
  return setMotorOutput(MotorDirection::Opening);
}

bool CurtainController::applyClose() {
  return setMotorOutput(MotorDirection::Closing);
}

void CurtainController::stop() {
  stopMotor();
}

void CurtainController::advancePosition(bool opening, unsigned long ms) {
  const float delta = (static_cast<float>(ms) / 1000.0f) * kPositionRate;
  if (opening) {
    current += delta;
    if (current > target) {
      current = target;
    }
  } else {
    current -= delta;
    if (current < target) {
      current = target;
    }
  }
  current = clamp01(current);
}

bool CurtainController::needOpen(float deadband) const {
  return target > current + deadband;
}

bool CurtainController::needClose(float deadband) const {
  return target < current - deadband;
}

uint8_t CurtainController::getOutputChannel() const {
  return outputChannel;
}

float CurtainController::clamp01(float value) const {
  if (value < 0.0f) {
    return 0.0f;
  }
  if (value > 1.0f) {
    return 1.0f;
  }
  return value;
}

bool CurtainController::setMotorOutput(MotorDirection direction) {
  if (!outputChannelValid || !IoHal::isOutputAvailable(outputChannel)) {
    return false;
  }
  const bool openDirection = direction == MotorDirection::Opening;
  const bool directionLevel = CurtainHardware::kDirectionHighIsOpen
                                  ? openDirection
                                  : !openDirection;
  IoHal::writeSharedSignalLevel(directionLevel, true);
  if (IoHal::writeOutput(outputChannel, true, true)) {
    motorEnabled = true;
    return true;
  } else {
    motorEnabled = false;
    LoggerService::warn("CurtainController", "output_enable_failed");
    return false;
  }
}

void CurtainController::stopMotor() {
  if (!outputChannelValid) {
    return;
  }
  if (motorEnabled) {
    (void)IoHal::writeOutput(outputChannel, false, false);
    motorEnabled = false;
  }
  if (!IoHal::isAnyOutputEnabled()) {
    IoHal::writeSharedSignalLevel(false, false);
  }
}
