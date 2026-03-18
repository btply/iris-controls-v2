#line 1 "/home/billy/Documents/Code/iris-controls-v2/src/core/SharedState.cpp"
#include "SharedState.h"

void SharedState::reset() {
  stateMutex.lock();
  weather = WeatherSnapshot();
  for (uint8_t i = 0; i < SharedStateConfig::kCwtCount; i++) {
    cwt[i] = CwtSnapshot();
  }
  currentIntent = ClimateIntent();
  currentPlan = CurtainPlan();
  lastPlanUpdateMs = 0UL;
  stateMutex.unlock();
}

void SharedState::updateWeather(const WeatherSnapshot& nextWeather) {
  stateMutex.lock();
  weather = nextWeather;
  stateMutex.unlock();
}

void SharedState::invalidateWeather() {
  stateMutex.lock();
  weather.valid = false;
  stateMutex.unlock();
}

void SharedState::updateCwt(uint8_t index, const CwtSnapshot& nextCwt) {
  if (index >= SharedStateConfig::kCwtCount) {
    return;
  }
  stateMutex.lock();
  cwt[index] = nextCwt;
  stateMutex.unlock();
}

void SharedState::invalidateCwt(uint8_t index) {
  if (index >= SharedStateConfig::kCwtCount) {
    return;
  }
  stateMutex.lock();
  cwt[index].valid = false;
  stateMutex.unlock();
}

void SharedState::invalidateAllCwt() {
  stateMutex.lock();
  for (uint8_t i = 0; i < SharedStateConfig::kCwtCount; i++) {
    cwt[i].valid = false;
  }
  stateMutex.unlock();
}

PlannerInput SharedState::copyPlannerInput(unsigned long nowMs) const {
  PlannerInput copy;
  copy.nowMs = nowMs;

  {
    stateMutex.lock();
    copy.weather = weather;
    for (uint8_t i = 0; i < SharedStateConfig::kCwtCount; i++) {
      copy.cwt[i] = cwt[i];
    }
    stateMutex.unlock();
  }

  float tempSum = 0.0f;
  float rhSum = 0.0f;
  uint8_t validCount = 0;
  for (uint8_t i = 0; i < SharedStateConfig::kCwtCount; i++) {
    if (!copy.cwt[i].valid) {
      continue;
    }
    tempSum += copy.cwt[i].tempC;
    rhSum += copy.cwt[i].rhPct;
    validCount++;
  }

  if (validCount > 0U) {
    copy.hasCwtAverage = true;
    copy.cwtAverageTempC = tempSum / static_cast<float>(validCount);
    copy.cwtAverageRhPct = rhSum / static_cast<float>(validCount);
  }

  return copy;
}

TelemetrySnapshot SharedState::copyTelemetry(unsigned long nowMs) const {
  TelemetrySnapshot telemetry;
  telemetry.nowMs = nowMs;
  stateMutex.lock();
  telemetry.weather = weather;
  for (uint8_t i = 0; i < SharedStateConfig::kCwtCount; i++) {
    telemetry.cwt[i] = cwt[i];
  }
  telemetry.intent = currentIntent;
  telemetry.plan = currentPlan;
  stateMutex.unlock();
  return telemetry;
}

void SharedState::applyPlan(const CurtainPlan& plan,
                            const ClimateIntent& intent,
                            unsigned long nowMs) {
  stateMutex.lock();
  currentPlan = plan;
  currentIntent = intent;
  lastPlanUpdateMs = nowMs;
  stateMutex.unlock();
}
