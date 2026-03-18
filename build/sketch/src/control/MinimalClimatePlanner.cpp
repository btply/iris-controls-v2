#line 1 "/home/billy/Documents/Code/iris-controls-v2/src/control/MinimalClimatePlanner.cpp"
#include "MinimalClimatePlanner.h"

#include "../config/SystemConfig.h"

ClimateIntent MinimalClimatePlanner::computeIntent(const PlannerInput& input,
                                                   unsigned long nowMs) {
  ClimateIntent intent;

  if (input.weather.valid) {
    if (input.weather.windSpeedMps >= SystemConfig::kEmergencyWindMps) {
      if (windAboveSinceMs == 0UL) {
        windAboveSinceMs = nowMs;
      }
      if (nowMs - windAboveSinceMs >= SystemConfig::kEmergencyWindAssertMs) {
        windEmergencyLatched = true;
      }
      windBelowSinceMs = 0UL;
    } else if (windEmergencyLatched &&
               input.weather.windSpeedMps <=
                   SystemConfig::kEmergencyWindClearMps) {
      if (windBelowSinceMs == 0UL) {
        windBelowSinceMs = nowMs;
      }
      if (nowMs - windBelowSinceMs >= SystemConfig::kEmergencyWindClearMs) {
        windEmergencyLatched = false;
      }
      windAboveSinceMs = 0UL;
    } else {
      windAboveSinceMs = 0UL;
      windBelowSinceMs = 0UL;
    }
  }

  if (input.weather.valid && input.weather.rainDetected) {
    rainLatched = true;
    rainClearSinceMs = 0UL;
  } else if (rainLatched) {
    if (rainClearSinceMs == 0UL) {
      rainClearSinceMs = nowMs;
    }
    if (nowMs - rainClearSinceMs >= SystemConfig::kRainClearMs) {
      rainLatched = false;
    }
  }

  intent.emergencyClose = windEmergencyLatched || rainLatched;
  if (intent.emergencyClose) {
    intent.ventRequested = false;
    intent.targetPosition = 0.0f;
    return intent;
  }

  if (!input.hasCwtAverage) {
    intent.ventRequested = false;
    intent.targetPosition = SystemConfig::kHoldPosition;
    return intent;
  }

  const bool tempHigh = input.cwtAverageTempC > SystemConfig::kTargetTempC;
  const bool rhHigh = input.cwtAverageRhPct > SystemConfig::kTargetRhPct;
  intent.ventRequested = tempHigh || rhHigh;
  intent.targetPosition = intent.ventRequested ? SystemConfig::kVentOpenPosition
                                               : SystemConfig::kHoldPosition;
  return intent;
}
