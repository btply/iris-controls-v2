#line 1 "/home/billy/Documents/Code/iris-controls-v2/src/core/AppLifecycle.cpp"
#include "AppLifecycle.h"

#include "../hal/IoHalLite.h"
#include "../services/LoggerService.h"
#include <Arduino.h>
#include <OptaBlue.h>
#include <array>

void AppLifecycle::begin() {
  LoggerService::begin(115200UL, 1500UL);
  LoggerService::info("AppLifecycle", "startup");

  OptaController.begin();

  IoHal::begin();

  curtainService.begin();

  modbusService.begin();
  mqttService.begin();

  devices.weather.reset();
  if (!modbusService.registerDevice(
          devices.weather,
          ModbusService::DeviceRole::Weather,
          SystemConfig::kWeatherPollIntervalMs,
          0U)) {
    LoggerService::error("AppLifecycle", "register_weather_failed");
  }

  for (uint8_t i = 0; i < AppDataConfig::kCwtCount; i++) {
    devices.cwt[i].reset();
  }

  struct CwtPollConfig {
    uint8_t index;
    uint8_t slaveId;
  };

  // Commissioned CWT sensors currently on bus: slave IDs 2 and 3.
  static const CwtPollConfig kCommissionedCwt[] = {
      {0U, 2U},
      {1U, 3U},
  };

  for (const CwtPollConfig& config : kCommissionedCwt) {
    devices.cwt[config.index].setSlaveId(config.slaveId);
    if (!modbusService.registerDevice(devices.cwt[config.index],
                                      ModbusService::DeviceRole::Cwt,
                                      SystemConfig::kCwtPollIntervalMs,
                                      config.index)) {
      LoggerService::printf(LoggerService::Level::Error,
                            "AppLifecycle",
                            "register_cwt%u_failed",
                            static_cast<unsigned int>(config.index));
    }
  }

  modbusService.start();
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

  if (nowMs - lastControlTickMs >= SystemConfig::kControlTickIntervalMs) {
    runControlTick(nowMs);
    lastControlTickMs = nowMs;
  }

  if (nowMs - lastTelemetryTickMs >= SystemConfig::kTelemetryIntervalMs) {
    runTelemetryTick(nowMs);
    lastTelemetryTickMs = nowMs;
  }

}

void AppLifecycle::updateSupervisor(unsigned long nowMs) {
  lastModbusHealth = modbusService.getHealth();
  lastMqttHealth = mqttService.getHealth();

  if (lastModbusHealth.lastLoopMs == 0UL) {
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

  const bool weatherFresh = devices.weather.isFresh(nowMs, SystemConfig::kWeatherFreshMaxAgeMs);
  bool allCwtFresh = true;
  for (uint8_t i = 0; i < AppDataConfig::kCwtCount; i++) {
    if (!devices.cwt[i].isFresh(nowMs, SystemConfig::kCwtFreshMaxAgeMs)) {
      allCwtFresh = false;
      break;
    }
  }
  const bool hasFreshData = weatherFresh && allCwtFresh;
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
  lastPlan = curtainService.getPlanSnapshot();
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
