#line 1 "/home/billy/Documents/Code/iris-controls-v2/src/services/MqttServiceLite.cpp"
#include "MqttServiceLite.h"

#include "../config/SystemConfig.h"
#include "LoggerService.h"
#include <Arduino.h>
#include <stdio.h>

void MqttService::begin(SharedState* sharedStateIn) {
  sharedState = sharedStateIn;
  mqttClient.setServer(SystemConfig::kMqttBrokerHost, SystemConfig::kMqttBrokerPort);
}

void MqttService::start() {
  if (running.load() || sharedState == nullptr) {
    return;
  }
  running.store(true);
  workerThread.start(mbed::callback(this, &MqttService::runThread));
}

void MqttService::stop() {
  if (!running.load()) {
    return;
  }
  running.store(false);
  workerThread.join();
}

void MqttService::publishTelemetry(const TelemetrySnapshot& telemetry) {
  stateMutex.lock();
  pendingTelemetry = telemetry;
  telemetryPending = true;
  stateMutex.unlock();
}

void MqttService::setRuntimeStatus(const RuntimeStatus& runtimeStatusIn) {
  stateMutex.lock();
  runtimeStatus = runtimeStatusIn;
  stateMutex.unlock();
}

MqttService::Health MqttService::getHealth() const {
  Health health;
  stateMutex.lock();
  health.connected = mqttClient.connected();
  health.networkReady = networkReady.load();
  health.lastLoopMs = lastLoopMs;
  health.connectFailures = connectFailures;
  health.publishFailures = publishFailures;
  stateMutex.unlock();
  return health;
}

void MqttService::runThread() {
  while (running.load()) {
    const unsigned long nowMs = millis();
    stateMutex.lock();
    lastLoopMs = nowMs;
    stateMutex.unlock();
    serviceNetwork(nowMs);
    tryReconnect(nowMs);

    if (networkReady.load() && mqttClient.connected()) {
      mqttClient.loop();
      flushPendingTelemetry();
    }

    rtos::ThisThread::sleep_for(SystemConfig::kMqttLoopSleepMs);
  }
}

void MqttService::initNetwork() {
  if (Ethernet.begin(SystemConfig::kEthernetMac) == 0) {
    networkReady.store(false);
    LoggerService::warn("Network", "dhcp_failed");
    return;
  }

  networkReady.store(true);
  LoggerService::info("Network", "dhcp_ready");
}

void MqttService::serviceNetwork(unsigned long nowMs) {
  if (!networkReady.load()) {
    if (nowMs - lastNetworkRetryMs < SystemConfig::kNetworkRetryIntervalMs) {
      return;
    }
    lastNetworkRetryMs = nowMs;
    initNetwork();
    return;
  }

  const int maintainResult = Ethernet.maintain();
  switch (maintainResult) {
    case 0:
      return;
    case 1:
    case 3:
      networkReady.store(false);
      if (mqttClient.connected()) {
        mqttClient.disconnect();
      }
      LoggerService::warn("Network", "dhcp_lost");
      return;
    case 2:
    case 4:
      LoggerService::warn("Network", "dhcp_renew_failed");
      return;
    default:
      return;
  }
}

void MqttService::tryReconnect(unsigned long nowMs) {
  if (mqttClient.connected()) {
    return;
  }
  if (!networkReady.load()) {
    return;
  }
  if (nowMs - lastReconnectAttemptMs < reconnectBackoffMs) {
    return;
  }

  lastReconnectAttemptMs = nowMs;
  if (mqttClient.connect(SystemConfig::kMqttClientId,
                         SystemConfig::kMqttUsername,
                         SystemConfig::kMqttPassword)) {
    LoggerService::info("MqttService", "connected");
    reconnectBackoffMs = SystemConfig::kMqttReconnectBackoffMinMs;
    return;
  }

  LoggerService::warn("MqttService", "connect_failed");
  stateMutex.lock();
  connectFailures++;
  stateMutex.unlock();
  reconnectBackoffMs *= 2UL;
  if (reconnectBackoffMs > SystemConfig::kMqttReconnectBackoffMaxMs) {
    reconnectBackoffMs = SystemConfig::kMqttReconnectBackoffMaxMs;
  }
}

void MqttService::flushPendingTelemetry() {
  TelemetrySnapshot telemetry;
  bool localPending = false;
  {
    stateMutex.lock();
    localPending = telemetryPending;
    if (localPending) {
      telemetry = pendingTelemetry;
      telemetryPending = false;
    }
    stateMutex.unlock();
  }

  if (!localPending || !mqttClient.connected()) {
    return;
  }

  char payload[512];
  if (!buildTelemetryPayload(payload, sizeof(payload), telemetry)) {
    return;
  }

  if (!mqttClient.publish(SystemConfig::kMqttTelemetryTopic, payload, false)) {
    LoggerService::warn("MqttService", "publish_failed");
    stateMutex.lock();
    publishFailures++;
    pendingTelemetry = telemetry;
    telemetryPending = true;
    stateMutex.unlock();
  }
}

bool MqttService::buildTelemetryPayload(char* out,
                                        size_t outSize,
                                        const TelemetrySnapshot& telemetry) const {
  if (out == nullptr || outSize < 64U) {
    return false;
  }

  RuntimeStatus localRuntimeStatus;
  unsigned long localMqttLastLoopMs = 0UL;
  unsigned long localConnectFailures = 0UL;
  unsigned long localPublishFailures = 0UL;
  const bool localMqttConnected = mqttClient.connected();
  const bool localNetworkReady = networkReady.load();
  {
    stateMutex.lock();
    localRuntimeStatus = runtimeStatus;
    localMqttLastLoopMs = lastLoopMs;
    localConnectFailures = connectFailures;
    localPublishFailures = publishFailures;
    stateMutex.unlock();
  }

  float avgTempC = 0.0f;
  float avgRhPct = 0.0f;
  uint8_t count = 0U;
  for (uint8_t i = 0; i < SharedStateConfig::kCwtCount; i++) {
    if (!telemetry.cwt[i].valid) {
      continue;
    }
    avgTempC += telemetry.cwt[i].tempC;
    avgRhPct += telemetry.cwt[i].rhPct;
    count++;
  }
  if (count > 0U) {
    avgTempC /= static_cast<float>(count);
    avgRhPct /= static_cast<float>(count);
  }

  const unsigned long modbusLoopAgeMs =
      telemetry.nowMs >= localRuntimeStatus.modbusLastLoopMs
          ? telemetry.nowMs - localRuntimeStatus.modbusLastLoopMs
          : 0UL;
  const unsigned long mqttLoopAgeMs =
      telemetry.nowMs >= localMqttLastLoopMs ? telemetry.nowMs - localMqttLastLoopMs
                                             : 0UL;

  const int written = snprintf(
      out, outSize,
      "{\"ts_ms\":%lu,\"wind_mps\":%.2f,\"rain\":%d,\"avg_temp_c\":%.2f,"
      "\"avg_rh_pct\":%.2f,\"target_pos\":%.2f,\"emergency_close\":%d,"
      "\"supervisor_state\":%u,\"control_enabled\":%d,\"modbus_weather_fresh\":%d,"
      "\"modbus_cwt_fresh\":%d,\"modbus_degraded\":%d,\"modbus_watchdog_fault\":%d,"
      "\"modbus_loop_age_ms\":%lu,\"mqtt_loop_age_ms\":%lu,"
      "\"modbus_read_failures\":%lu,\"modbus_weather_failures\":%lu,"
      "\"modbus_cwt_failures\":%lu,\"mqtt_connected\":%d,\"network_ready\":%d,"
      "\"mqtt_connect_failures\":%lu,\"mqtt_publish_failures\":%lu,"
      "\"io_expansion_bound\":%d,"
      "\"io_write_healthy\":%d,\"io_output_uncertain\":%d,"
      "\"io_write_failures\":%lu,\"io_consecutive_failures\":%lu}",
      telemetry.nowMs, telemetry.weather.windSpeedMps,
      telemetry.weather.rainDetected ? 1 : 0, avgTempC, avgRhPct,
      telemetry.intent.targetPosition, telemetry.intent.emergencyClose ? 1 : 0,
      static_cast<unsigned int>(localRuntimeStatus.supervisorState),
      localRuntimeStatus.controlEnabled ? 1 : 0,
      localRuntimeStatus.modbusWeatherFresh ? 1 : 0,
      localRuntimeStatus.modbusCwtFresh ? 1 : 0,
      localRuntimeStatus.modbusDegraded ? 1 : 0,
      localRuntimeStatus.modbusWatchdogFault ? 1 : 0,
      modbusLoopAgeMs,
      mqttLoopAgeMs,
      localRuntimeStatus.modbusTotalReadFailures,
      localRuntimeStatus.modbusConsecutiveWeatherFailures,
      localRuntimeStatus.modbusConsecutiveCwtFailures,
      localMqttConnected ? 1 : 0,
      localNetworkReady ? 1 : 0,
      localConnectFailures,
      localPublishFailures,
      localRuntimeStatus.ioExpansionBound ? 1 : 0,
      localRuntimeStatus.ioExpansionWriteHealthy ? 1 : 0,
      localRuntimeStatus.ioExpansionOutputStateUncertain ? 1 : 0,
      localRuntimeStatus.ioExpansionWriteFailures,
      localRuntimeStatus.ioExpansionConsecutiveFailures);
  return written > 0 && static_cast<size_t>(written) < outSize;
}
