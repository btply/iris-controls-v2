#ifndef APP_DATA_TYPES_H
#define APP_DATA_TYPES_H

#include <stdint.h>

namespace AppDataConfig {
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
  CwtSnapshot cwt[AppDataConfig::kCwtCount];
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
  float targetByCurtain[AppDataConfig::kCurtainCount] = {0.0f, 0.0f, 0.0f, 0.0f};
};

struct TelemetrySnapshot {
  unsigned long nowMs = 0UL;
  WeatherSnapshot weather;
  CwtSnapshot cwt[AppDataConfig::kCwtCount];
  ClimateIntent intent;
  CurtainPlan plan;
};

#endif  // APP_DATA_TYPES_H
