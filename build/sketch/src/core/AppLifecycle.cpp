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

  sharedState.reset();
  OptaController.begin();
  IoHal::begin();
  curtainService.begin();

  modbusService.begin(&sharedState);
  mqttService.begin(&sharedState);
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
  lastControlTickMs = millis();
  lastTelemetryTickMs = millis();
}

void AppLifecycle::tick() {
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
  lastModbusHealth = modbusService.getHealth(nowMs);
  lastMqttHealth = mqttService.getHealth();

  if (lastModbusHealth.lastLoopMs == 0UL) {
    modbusWatchdogFault = true;
  } else {
    modbusWatchdogFault =
        nowMs - lastModbusHealth.lastLoopMs > SystemConfig::kServiceHeartbeatTimeoutMs;
  }
  if (lastMqttHealth.lastLoopMs == 0UL) {
    mqttWatchdogFault = true;
  } else {
    mqttWatchdogFault =
        nowMs - lastMqttHealth.lastLoopMs > SystemConfig::kServiceHeartbeatTimeoutMs;
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

  const bool hasFreshData = modbusService.hasFreshData(nowMs);
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
  const PlannerInput plannerInput = sharedState.copyPlannerInput(nowMs);
  ClimateIntent intent;
  if (controlEnabled) {
    intent = climatePlanner.computeIntent(plannerInput, nowMs);
  } else {
    intent.emergencyClose = true;
    intent.ventRequested = false;
    intent.targetPosition = 0.0f;
  }

  std::array<float, SharedStateConfig::kCurtainCount> targets = {intent.targetPosition,
                                                                  intent.targetPosition,
                                                                  intent.targetPosition,
                                                                  intent.targetPosition};
  if (intent.emergencyClose) {
    targets = {0.0f, 0.0f, 0.0f, 0.0f};
  }
  curtainService.setTargets(targets, intent.emergencyClose);
  const CurtainPlan plan = curtainService.getPlanSnapshot();
  sharedState.applyPlan(plan, intent, nowMs);
}

void AppLifecycle::runTelemetryTick(unsigned long nowMs) {
  const TelemetrySnapshot telemetry = sharedState.copyTelemetry(nowMs);
  const IoHal::Status ioStatus = IoHal::getStatus();

  MqttService::RuntimeStatus runtimeStatus;
  runtimeStatus.supervisorState = static_cast<uint8_t>(supervisorState);
  runtimeStatus.controlEnabled = controlEnabled;
  runtimeStatus.modbusWeatherFresh = lastModbusHealth.weatherFresh;
  runtimeStatus.modbusCwtFresh = lastModbusHealth.cwtFresh;
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
