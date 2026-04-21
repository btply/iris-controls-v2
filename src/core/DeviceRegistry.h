#ifndef DEVICE_REGISTRY_H
#define DEVICE_REGISTRY_H

#include "AppDataTypes.h"
#include "../devices/CwtDevice.h"
#include "../devices/WeatherDevice.h"

/** Owns concrete Modbus device instances; commissioned list lives in devices/ModbusCommissioningTable.h */
struct DeviceRegistry {
  WeatherDevice weather;
  CwtDevice cwt[AppDataConfig::kCwtCount];
};

#endif  // DEVICE_REGISTRY_H
