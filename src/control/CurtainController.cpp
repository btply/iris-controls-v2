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
  motorEnabled = false;
  stopMotor();
}

void CurtainController::setTarget(float targetPositionIn) {
  target = clamp01(targetPositionIn);
}

void CurtainController::emergencyClose() {
  target = 0.0f;
}

float CurtainController::currentPosition() const {
  return current;
}

float CurtainController::targetPosition() const {
  return target;
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
  }
  motorEnabled = false;
  LoggerService::warn("CurtainController", "output_enable_failed");
  return false;
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
