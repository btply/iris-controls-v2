#line 1 "/home/billy/Documents/Code/iris-controls-v2/src/control/CurtainController.h"
#ifndef CURTAIN_CONTROLLER_H
#define CURTAIN_CONTROLLER_H

#include <stdint.h>

class CurtainController {
 public:
  explicit CurtainController(uint8_t curtainIdIn);

  void begin();
  void tick(unsigned long nowMs);

  void setTarget(float targetPositionIn);
  void emergencyClose();

  bool applyOpen();
  bool applyClose();
  void stop();

  void advancePosition(bool opening, unsigned long ms);

  bool needOpen(float deadband) const;
  bool needClose(float deadband) const;

  float currentPosition() const;
  float targetPosition() const;
  bool isEmergencyClosed() const;
  uint8_t getOutputChannel() const;

 private:
  enum class MotorDirection : uint8_t {
    Opening = 0,
    Closing = 1,
  };

  float clamp01(float value) const;
  bool setMotorOutput(MotorDirection direction);
  void stopMotor();

  static const float kPositionRate;

  uint8_t curtainId;
  uint8_t outputChannel = 0U;
  bool outputChannelValid = false;
  float current = 0.0f;
  float target = 0.0f;
  bool emergencyClosed = false;
  bool motorEnabled = false;
  unsigned long lastTickMs = 0UL;
};

#endif  // CURTAIN_CONTROLLER_H
