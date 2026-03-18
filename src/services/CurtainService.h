#ifndef CURTAIN_SERVICE_H
#define CURTAIN_SERVICE_H

#include "../control/CurtainController.h"
#include "../core/SharedState.h"
#include <array>
#include <atomic>
#include <mbed.h>

class CurtainService {
 public:
  struct Health {
    bool running = false;
    unsigned long lastLoopMs = 0UL;
  };

  void begin();
  void start();
  void stop();

  void setTargets(const std::array<float, SharedStateConfig::kCurtainCount>& targets,
                  bool emergencyClose);
  CurtainPlan getPlanSnapshot() const;
  Health getHealth() const;

 private:
  void runThread();
  void onCurtainTimeout(uint8_t index);
  void armCurtainTimeout(uint8_t index, unsigned long runMs);
  void disarmAllCurtainTimeouts();

  void onCurtainTimeout0();
  void onCurtainTimeout1();
  void onCurtainTimeout2();
  void onCurtainTimeout3();

  rtos::Thread workerThread;
  std::atomic<bool> running{false};

  mutable rtos::Mutex commandMutex;
  std::array<float, SharedStateConfig::kCurtainCount> desiredTargets = {0.0f, 0.0f, 0.0f,
                                                                         0.0f};
  bool desiredEmergencyClose = false;

  mutable rtos::Mutex stateMutex;
  CurtainPlan currentPlan;
  unsigned long lastLoopMs = 0UL;

  CurtainController controllers[SharedStateConfig::kCurtainCount] = {
      CurtainController(0), CurtainController(1), CurtainController(2),
      CurtainController(3)};

  mbed::Timeout movementTimeouts[SharedStateConfig::kCurtainCount];
};

#endif  // CURTAIN_SERVICE_H
