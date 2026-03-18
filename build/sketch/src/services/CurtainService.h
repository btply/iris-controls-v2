#line 1 "/home/billy/Documents/Code/iris-controls-v2/src/services/CurtainService.h"
#ifndef CURTAIN_SERVICE_H
#define CURTAIN_SERVICE_H

#include "../control/CurtainController.h"
#include "../core/AppDataTypes.h"
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

  void setTargets(const std::array<float, AppDataConfig::kCurtainCount>& targets,
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
  std::array<float, AppDataConfig::kCurtainCount> desiredTargets = {0.0f, 0.0f, 0.0f, 0.0f};
  bool desiredEmergencyClose = false;

  mutable rtos::Mutex stateMutex;
  CurtainPlan currentPlan;
  unsigned long lastLoopMs = 0UL;

  CurtainController controllers[AppDataConfig::kCurtainCount] = {
      CurtainController(0), CurtainController(1), CurtainController(2),
      CurtainController(3)};

  mbed::Timeout movementTimeouts[AppDataConfig::kCurtainCount];
};

#endif  // CURTAIN_SERVICE_H
