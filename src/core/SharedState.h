#ifndef SHARED_STATE_H
#define SHARED_STATE_H

#include <mbed.h>
#include <stdint.h>

namespace SharedStateConfig {
static const uint8_t kCurtainCount = 4U;
static const uint8_t kCwtCount = 4U;
}

struct WeatherSnapshot {
  bool valid = false;
  bool rainDetected = false;
  float windSpeedMps = 0.0f;
  unsigned long lastUpdateMs = 0UL;
};

struct CwtSnapshot {
  bool valid = false;
  float tempC = 0.0f;
  float rhPct = 0.0f;
  unsigned long lastUpdateMs = 0UL;
};

struct PlannerInput {
  WeatherSnapshot weather;
  CwtSnapshot cwt[SharedStateConfig::kCwtCount];
  bool hasCwtAverage = false;
  float cwtAverageTempC = 0.0f;
  float cwtAverageRhPct = 0.0f;
  unsigned long nowMs = 0UL;
};

struct ClimateIntent {
  bool emergencyClose = false;
  bool ventRequested = false;
  float targetPosition = 0.0f;
};

struct CurtainPlan {
  bool valid = false;
  bool emergencyClose = false;
  float targetByCurtain[SharedStateConfig::kCurtainCount] = {0.0f, 0.0f, 0.0f,
                                                             0.0f};
};

struct TelemetrySnapshot {
  unsigned long nowMs = 0UL;
  WeatherSnapshot weather;
  CwtSnapshot cwt[SharedStateConfig::kCwtCount];
  ClimateIntent intent;
  CurtainPlan plan;
};

class SharedState {
 public:
  void reset();

  void updateWeather(const WeatherSnapshot& weather);
  void updateCwt(uint8_t index, const CwtSnapshot& cwt);
  void invalidateWeather();
  void invalidateCwt(uint8_t index);
  void invalidateAllCwt();

  PlannerInput copyPlannerInput(unsigned long nowMs) const;
  TelemetrySnapshot copyTelemetry(unsigned long nowMs) const;

  void applyPlan(const CurtainPlan& plan,
                 const ClimateIntent& intent,
                 unsigned long nowMs);

 private:
  // Rule: keep lock scope short and never call blocking I/O while locked.
  mutable rtos::Mutex stateMutex;
  WeatherSnapshot weather;
  CwtSnapshot cwt[SharedStateConfig::kCwtCount];
  ClimateIntent currentIntent;
  CurtainPlan currentPlan;
  unsigned long lastPlanUpdateMs = 0UL;
};

#endif  // SHARED_STATE_H
