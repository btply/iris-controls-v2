#include <Arduino.h>
#include <OptaBlue.h>
#include <drivers/Watchdog.h>

#include "AppLifecycle.h"
#include "../config/SystemConfig.h"
#include "../hal/IoHalLite.h"
#include "../services/LoggerService.h"

#include <array>
#include <chrono>
#include <mbed.h>

void AppLifecycle::begin() {
  LoggerService::begin(115200UL, 1500UL);
  LoggerService::info("AppLifecycle", "startup");

  mbed::Watchdog::get_instance().start(8000);

  OptaController.begin();

  IoHal::begin();

  curtainService.begin();

  mqttService.begin();

  for (uint8_t i = 0; i < AppDataConfig::kCwtCount; i++) {
    devices.cwt[i].reset();
  }
  devices.weather.reset();

  modbusService.begin(devices);
  mqttService.start();

  curtainService.start();
  LoggerService::info("AppLifecycle", "services_started");

  supervisorState = SupervisorState::PreOperational;
  controlEnabled = false;
  modbusWatchdogFault = false;
  mqttWatchdogFault = false;
  modbusWatchdogFaultLogged = false;
  mqttWatchdogFaultLogged = false;
  lastIntent = ClimateIntent{};
  lastPlan = CurtainPlan{};
  lastControlTickMs = millis();
  lastTelemetryTickMs = millis();
}

void AppLifecycle::tick() {
  LoggerService::drain();
  const unsigned long nowMs = millis();

  IoHal::update();
  updateSupervisor(nowMs);
  mbed::Watchdog::get_instance().kick();

  if (nowMs - lastControlTickMs >= SystemConfig::kControlTickIntervalMs) {
    runControlTick(nowMs);
    lastControlTickMs = nowMs;
  }

  if (nowMs - lastTelemetryTickMs >= SystemConfig::kTelemetryIntervalMs) {
    runTelemetryTick(nowMs);
    lastTelemetryTickMs = nowMs;
  }

  rtos::ThisThread::sleep_for(std::chrono::milliseconds(5));
}

void AppLifecycle::updateSupervisor(unsigned long nowMs) {
  lastModbusHealth = modbusService.getHealth();
  lastMqttHealth = mqttService.getHealth();

  if (!lastModbusHealth.pollThreadRunning) {
    modbusWatchdogFault = false;
  } else if (lastModbusHealth.lastLoopMs == 0UL) {
    modbusWatchdogFault = true;
  } else {
    modbusWatchdogFault = nowMs - lastModbusHealth.lastLoopMs >
                          SystemConfig::kModbusServiceHeartbeatTimeoutMs;
  }
  if (lastMqttHealth.lastLoopMs == 0UL) {
    mqttWatchdogFault = true;
  } else {
    mqttWatchdogFault = nowMs - lastMqttHealth.lastLoopMs >
                        SystemConfig::kMqttServiceHeartbeatTimeoutMs;
  }

  if (modbusWatchdogFault && !modbusWatchdogFaultLogged) {
    LoggerService::warn("AppLifecycle", "modbus_watchdog_fault");
    modbusWatchdogFaultLogged = true;
  }
  if (!modbusWatchdogFault) {
    modbusWatchdogFaultLogged = false;
  }
  if (mqttWatchdogFault && !mqttWatchdogFaultLogged) {
    LoggerService::warn("AppLifecycle", "mqtt_watchdog_fault");
    mqttWatchdogFaultLogged = true;
  }
  if (!mqttWatchdogFault) {
    mqttWatchdogFaultLogged = false;
  }

  const bool weatherOk =
      SystemConfig::kSkipWeatherPolling ||
      devices.weather.isFresh(nowMs, SystemConfig::kWeatherFreshMaxAgeMs);
  bool allCwtFresh = true;
  for (uint8_t i = 0; i < AppDataConfig::kCwtCount; i++) {
    if (!devices.cwt[i].isFresh(nowMs, SystemConfig::kCwtFreshMaxAgeMs)) {
      allCwtFresh = false;
      break;
    }
  }
  const bool hasFreshData = weatherOk && allCwtFresh;
  const bool hasServiceFaults =
      modbusWatchdogFault || mqttWatchdogFault || lastModbusHealth.degraded;

  if (supervisorState == SupervisorState::PreOperational && hasFreshData &&
      !hasServiceFaults) {
    supervisorState = SupervisorState::Operational;
    controlEnabled = true;
    LoggerService::info("AppLifecycle", "operational");
    return;
  }

  if (supervisorState == SupervisorState::Operational &&
      (!hasFreshData || hasServiceFaults)) {
    supervisorState = SupervisorState::Degraded;
    controlEnabled = false;
    LoggerService::warn("AppLifecycle", "degraded");
    return;
  }

  if (supervisorState == SupervisorState::Degraded && hasFreshData &&
      !hasServiceFaults) {
    supervisorState = SupervisorState::Operational;
    controlEnabled = true;
    LoggerService::info("AppLifecycle", "recovered");
  }
}

void AppLifecycle::runControlTick(unsigned long nowMs) {
  lastPlan = curtainService.getPlanSnapshot();

  PlannerInput plannerInput;
  plannerInput.nowMs = nowMs;
  plannerInput.weather = devices.weather.getSnapshot();
  float tempSum = 0.0f;
  float rhSum = 0.0f;
  uint8_t validCount = 0U;
  for (uint8_t i = 0; i < AppDataConfig::kCwtCount; i++) {
    plannerInput.cwt[i] = devices.cwt[i].getSnapshot();
    if (!plannerInput.cwt[i].valid) {
      continue;
    }
    tempSum += plannerInput.cwt[i].tempC;
    rhSum += plannerInput.cwt[i].rhPct;
    validCount++;
  }
  if (validCount > 0U) {
    plannerInput.hasCwtAverage = true;
    plannerInput.cwtAverageTempC = tempSum / static_cast<float>(validCount);
    plannerInput.cwtAverageRhPct = rhSum / static_cast<float>(validCount);
  }

  ClimateIntent intent;
  if (controlEnabled) {
    intent = climatePlanner.computeIntent(plannerInput, nowMs);
  } else {
    intent.emergencyClose = true;
    intent.ventRequested = false;
    intent.targetPosition = 0.0f;
  }
  lastIntent = intent;

  std::array<float, AppDataConfig::kCurtainCount> targets = {intent.targetPosition,
                                                              intent.targetPosition,
                                                              intent.targetPosition,
                                                              intent.targetPosition};
  if (intent.emergencyClose) {
    targets = {0.0f, 0.0f, 0.0f, 0.0f};
  }
  curtainService.setTargets(targets, intent.emergencyClose);
}

void AppLifecycle::runTelemetryTick(unsigned long nowMs) {
  TelemetrySnapshot telemetry;
  telemetry.nowMs = nowMs;
  telemetry.weather = devices.weather.getSnapshot();
  for (uint8_t i = 0; i < AppDataConfig::kCwtCount; i++) {
    telemetry.cwt[i] = devices.cwt[i].getSnapshot();
  }
  telemetry.intent = lastIntent;
  telemetry.plan = lastPlan;

  const IoHal::Status ioStatus = IoHal::getStatus();
  bool allCwtFresh = true;
  for (uint8_t i = 0; i < AppDataConfig::kCwtCount; i++) {
    if (!devices.cwt[i].isFresh(nowMs, SystemConfig::kCwtFreshMaxAgeMs)) {
      allCwtFresh = false;
      break;
    }
  }

  MqttService::RuntimeStatus runtimeStatus;
  runtimeStatus.supervisorState = static_cast<uint8_t>(supervisorState);
  runtimeStatus.controlEnabled = controlEnabled;
  runtimeStatus.modbusWeatherFresh =
      SystemConfig::kSkipWeatherPolling ||
      devices.weather.isFresh(nowMs, SystemConfig::kWeatherFreshMaxAgeMs);
  runtimeStatus.modbusCwtFresh = allCwtFresh;
  runtimeStatus.modbusDegraded = lastModbusHealth.degraded || modbusWatchdogFault;
  runtimeStatus.modbusWatchdogFault = modbusWatchdogFault;
  runtimeStatus.modbusLastLoopMs = lastModbusHealth.lastLoopMs;
  runtimeStatus.modbusTotalReadFailures = lastModbusHealth.totalReadFailures;
  runtimeStatus.modbusConsecutiveWeatherFailures =
      lastModbusHealth.weatherConsecutiveFailures;
  runtimeStatus.modbusConsecutiveCwtFailures = lastModbusHealth.cwtConsecutiveFailures;
  runtimeStatus.ioExpansionBound = ioStatus.expansionBound;
  runtimeStatus.ioExpansionWriteHealthy = ioStatus.expansionWriteHealthy;
  runtimeStatus.ioExpansionOutputStateUncertain = ioStatus.expansionOutputStateUncertain;
  runtimeStatus.ioExpansionWriteFailures = ioStatus.expansionWriteFailures;
  runtimeStatus.ioExpansionConsecutiveFailures = ioStatus.expansionConsecutiveFailures;
  mqttService.setRuntimeStatus(runtimeStatus);

  mqttService.publishTelemetry(telemetry);
}
