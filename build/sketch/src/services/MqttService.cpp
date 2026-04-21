#line 1 "/home/billy/Documents/Code/iris-controls-v2/src/services/MqttService.cpp"
#include "MqttService.h"

#include "../config/SystemConfig.h"
#include "LoggerService.h"
#include <Arduino.h>
#include <mbed.h>
#include <stdio.h>
#include <chrono>

void MqttService::begin() {
  mqttClient.setServer(SystemConfig::kMqttBrokerHost, SystemConfig::kMqttBrokerPort);
  mqttClient.setBufferSize(640);
  mqttClient.setKeepAlive(30);
}

void MqttService::start() {
  if (running.load()) {
    return;
  }
  running.store(true);
  const osStatus st = workerThread.start(mbed::callback(this, &MqttService::runThread));
  if (st != osOK) {
    running.store(false);
    LoggerService::enqueue(LoggerService::Level::Error, "MqttService", "thread_start_failed");
  }
}

void MqttService::stop() {
  if (!running.load()) {
    return;
  }
  running.store(false);
  workerThread.join();
}

void MqttService::publishTelemetry(const TelemetrySnapshot& telemetry) {
  mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
  pendingTelemetry = telemetry;
  telemetryPending = true;
}

void MqttService::setRuntimeStatus(const RuntimeStatus& runtimeStatusIn) {
  mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
  runtimeStatus = runtimeStatusIn;
}

MqttService::Health MqttService::getHealth() const {
  Health health;
  mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
  health.connected = mqttConnectedCached.load();
  health.networkReady = networkReady.load();
  health.lastLoopMs = lastLoopMs;
  health.connectFailures = connectFailures;
  health.publishFailures = publishFailures;
  return health;
}

void MqttService::touchHeartbeat() {
  mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
  lastLoopMs = millis();
}

void MqttService::runThread() {
  while (running.load()) {
    touchHeartbeat();
    const unsigned long nowMs = millis();
    touchHeartbeat();
    serviceNetwork(nowMs);
    touchHeartbeat();
    tryReconnect(nowMs);

    const bool netOk = networkReady.load();
    const bool mqttOk = mqttClient.connected();
    mqttConnectedCached.store(mqttOk);

    if (netOk && mqttOk) {
      touchHeartbeat();
      mqttClient.loop();
      touchHeartbeat();
      flushPendingTelemetry();
      touchHeartbeat();
    }

    rtos::ThisThread::sleep_for(std::chrono::milliseconds(SystemConfig::kMqttLoopSleepMs));
  }
  mqttConnectedCached.store(false);
}

void MqttService::initNetwork() {
  touchHeartbeat();
  if (Ethernet.begin(SystemConfig::kEthernetMac) == 0) {
    touchHeartbeat();
    networkReady.store(false);
    LoggerService::enqueue(LoggerService::Level::Warn, "Network", "dhcp_failed");
    return;
  }

  touchHeartbeat();
  networkReady.store(true);
  LoggerService::enqueue(LoggerService::Level::Info, "Network", "dhcp_ready");
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

  touchHeartbeat();
  const int maintainResult = Ethernet.maintain();
  touchHeartbeat();
  switch (maintainResult) {
    case 0:
      return;
    case 1:
    case 3:
      networkReady.store(false);
      if (mqttClient.connected()) {
        mqttClient.disconnect();
      }
      mqttConnectedCached.store(false);
      LoggerService::enqueue(LoggerService::Level::Warn, "Network", "dhcp_lost");
      return;
    case 2:
    case 4:
      LoggerService::enqueue(LoggerService::Level::Warn, "Network", "dhcp_renew_failed");
      return;
    default:
      return;
  }
}

void MqttService::tryReconnect(unsigned long nowMs) {
  if (mqttClient.connected()) {
    return;
  }
  mqttConnectedCached.store(false);
  if (!networkReady.load()) {
    return;
  }
  if (nowMs - lastReconnectAttemptMs < reconnectBackoffMs) {
    return;
  }

  lastReconnectAttemptMs = nowMs;
  touchHeartbeat();
  if (mqttClient.connect(SystemConfig::kMqttClientId,
                         SystemConfig::kMqttUsername,
                         SystemConfig::kMqttPassword)) {
    touchHeartbeat();
    mqttConnectedCached.store(true);
    LoggerService::enqueue(LoggerService::Level::Info, "MqttService", "connected");
    reconnectBackoffMs = SystemConfig::kMqttReconnectBackoffMinMs;
    return;
  }
  touchHeartbeat();

  LoggerService::enqueue(LoggerService::Level::Warn, "MqttService", "connect_failed");
  {
    mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
    connectFailures++;
  }
  reconnectBackoffMs *= 2UL;
  if (reconnectBackoffMs > SystemConfig::kMqttReconnectBackoffMaxMs) {
    reconnectBackoffMs = SystemConfig::kMqttReconnectBackoffMaxMs;
  }
}

void MqttService::flushPendingTelemetry() {
  TelemetrySnapshot telemetry;
  bool localPending = false;
  {
    mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
    localPending = telemetryPending;
    if (localPending) {
      telemetry = pendingTelemetry;
      telemetryPending = false;
    }
  }

  if (!localPending || !mqttClient.connected()) {
    return;
  }

  char payload[512];
  if (!buildTelemetryPayload(payload, sizeof(payload), telemetry)) {
    return;
  }

  touchHeartbeat();
  if (!mqttClient.publish(SystemConfig::kMqttTelemetryTopic, payload, false)) {
    touchHeartbeat();
    LoggerService::enqueue(LoggerService::Level::Warn, "MqttService", "publish_failed");
    {
      mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
      publishFailures++;
      pendingTelemetry = telemetry;
      telemetryPending = true;
    }
    return;
  }
  touchHeartbeat();
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
  const bool localMqttConnected = mqttConnectedCached.load();
  const bool localNetworkReady = networkReady.load();
  {
    mbed::ScopedLock<rtos::Mutex> lock(stateMutex);
    localRuntimeStatus = runtimeStatus;
    localMqttLastLoopMs = lastLoopMs;
    localConnectFailures = connectFailures;
    localPublishFailures = publishFailures;
  }

  float avgTempC = 0.0f;
  float avgRhPct = 0.0f;
  uint8_t count = 0U;
  for (uint8_t i = 0; i < AppDataConfig::kCwtCount; i++) {
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
