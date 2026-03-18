#line 1 "/home/billy/Documents/Code/iris-controls-v2/src/core/AppLifecycle.h"
#ifndef APP_LIFECYCLE_H
#define APP_LIFECYCLE_H

#include "../config/SystemConfig.h"
#include "../control/MinimalClimatePlanner.h"
#include "../core/DeviceRegistry.h"
#include "../services/CurtainService.h"
#include "../services/ModbusService.h"
#include "../services/MqttServiceLite.h"

class AppLifecycle {
 public:
  void begin();
  void tick();

 private:
  enum class SupervisorState : uint8_t {
    BootInit = 0,
    PreOperational = 1,
    Operational = 2,
    Degraded = 3,
  };

  void updateSupervisor(unsigned long nowMs);
  void runControlTick(unsigned long nowMs);
  void runTelemetryTick(unsigned long nowMs);

  SupervisorState supervisorState = SupervisorState::BootInit;
  bool controlEnabled = false;
  bool modbusWatchdogFault = false;
  bool mqttWatchdogFault = false;
  bool modbusWatchdogFaultLogged = false;
  bool mqttWatchdogFaultLogged = false;
  unsigned long lastControlTickMs = 0UL;
  unsigned long lastTelemetryTickMs = 0UL;
  ModbusService::Health lastModbusHealth;
  MqttService::Health lastMqttHealth;
  ClimateIntent lastIntent;
  CurtainPlan lastPlan;

  DeviceRegistry devices;
  ModbusService modbusService;
  MqttService mqttService;
  CurtainService curtainService;
  MinimalClimatePlanner climatePlanner;
};

#endif  // APP_LIFECYCLE_H
