#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <stdint.h>

namespace SystemConfig {

static const unsigned long kControlTickIntervalMs = 2000UL;
static const unsigned long kTelemetryIntervalMs = 30000UL;

static const unsigned long kModbusLoopSleepMs = 20UL;
static const unsigned long kMqttLoopSleepMs = 20UL;
static const unsigned long kCurtainLoopSleepMs = 20UL;
static const unsigned long kCurtainOpenSliceMs = 10UL;
static const unsigned long kCurtainCloseSliceMs = 10UL;
static const float kCurtainPositionDeadband = 0.02f;
static const float kCurtainEmergencyCloseEpsilon = 0.001f;
static const unsigned long kMqttReconnectBackoffMinMs = 1000UL;
static const unsigned long kMqttReconnectBackoffMaxMs = 30000UL;

static const unsigned long kWeatherPollIntervalMs = 10000UL;
static const unsigned long kCwtPollIntervalMs = 5000UL;
static const unsigned long kWeatherFreshMaxAgeMs = 30000UL;
static const unsigned long kCwtFreshMaxAgeMs = 15000UL;
static const unsigned long kModbusServiceHeartbeatTimeoutMs = 3000UL;
static const unsigned long kMqttServiceHeartbeatTimeoutMs = 3000UL;
static const uint8_t kModbusFailureDegradeThreshold = 3U;

static const float kVentOpenPosition = 0.65f;
static const float kHoldPosition = 0.20f;
static const float kTargetTempC = 23.0f;
static const float kTargetRhPct = 78.0f;

static const float kEmergencyWindMps = 18.0f;
static const float kEmergencyWindClearMps = 12.0f;
static const unsigned long kEmergencyWindAssertMs = 2000UL;
static const unsigned long kEmergencyWindClearMs = 10000UL;
static const unsigned long kRainClearMs = 300000UL;

static const char* const kMqttBrokerHost = "192.168.1.100";
static const uint16_t kMqttBrokerPort = 1883U;
static const char* const kMqttClientId = "iris-controls-opta";
static const char* const kMqttUsername = "opta";
static const char* const kMqttPassword = "opta_password_change_me";
static const char* const kMqttTopicBase = "iris/wolds/tunnel1";
static const char* const kMqttTelemetryTopic = "iris/wolds/tunnel1/telemetry/v2";

static uint8_t kEthernetMac[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x11};
static const unsigned long kNetworkRetryIntervalMs = 5000UL;

}  // namespace SystemConfig

#endif  // SYSTEM_CONFIG_H
