#line 1 "/home/billy/Documents/Code/iris-controls-v2/src/services/MqttServiceLite.h"
#ifndef MQTT_SERVICE_LITE_H
#define MQTT_SERVICE_LITE_H

#include "../core/AppDataTypes.h"
#include <Ethernet.h>
#include <PubSubClient.h>
#include <atomic>
#include <mbed.h>
#include <stddef.h>

class MqttService {
 public:
  struct Health {
    bool connected = false;
    bool networkReady = false;
    unsigned long lastLoopMs = 0UL;
    unsigned long connectFailures = 0UL;
    unsigned long publishFailures = 0UL;
  };

  struct RuntimeStatus {
    uint8_t supervisorState = 0U;
    bool controlEnabled = false;
    bool modbusWeatherFresh = false;
    bool modbusCwtFresh = false;
    bool modbusDegraded = false;
    bool modbusWatchdogFault = false;
    unsigned long modbusLastLoopMs = 0UL;
    unsigned long modbusTotalReadFailures = 0UL;
    unsigned long modbusConsecutiveWeatherFailures = 0UL;
    unsigned long modbusConsecutiveCwtFailures = 0UL;
    bool ioExpansionBound = false;
    bool ioExpansionWriteHealthy = true;
    bool ioExpansionOutputStateUncertain = false;
    unsigned long ioExpansionWriteFailures = 0UL;
    unsigned long ioExpansionConsecutiveFailures = 0UL;
  };

  void begin();

  void start();
  void stop();

  void publishTelemetry(const TelemetrySnapshot& telemetry);
  void setRuntimeStatus(const RuntimeStatus& runtimeStatusIn);
  Health getHealth() const;

 private:
  void runThread();
  void initNetwork();
  void serviceNetwork(unsigned long nowMs);
  void tryReconnect(unsigned long nowMs);
  void touchHeartbeat();
  void flushPendingTelemetry();
  bool buildTelemetryPayload(char* out,
                             size_t outSize,
                             const TelemetrySnapshot& telemetry) const;

  EthernetClient ethernetClient;
  mutable PubSubClient mqttClient{ethernetClient};
  rtos::Thread workerThread;
  std::atomic<bool> running{false};
  std::atomic<bool> networkReady{false};

  mutable rtos::Mutex stateMutex;
  TelemetrySnapshot pendingTelemetry;
  bool telemetryPending = false;
  RuntimeStatus runtimeStatus;
  unsigned long lastNetworkRetryMs = 0UL;
  unsigned long lastReconnectAttemptMs = 0UL;
  unsigned long reconnectBackoffMs = 1000UL;
  unsigned long lastLoopMs = 0UL;
  unsigned long connectFailures = 0UL;
  unsigned long publishFailures = 0UL;
};

#endif  // MQTT_SERVICE_LITE_H
