#line 1 "/home/billy/Documents/Code/iris-controls-v2/src/core/DeviceRegistry.h"
#ifndef DEVICE_REGISTRY_H
#define DEVICE_REGISTRY_H

#include "AppDataTypes.h"
#include "../devices/CwtDevice.h"
#include "../devices/WeatherDevice.h"

struct DeviceRegistry {
  WeatherDevice weather;
  CwtDevice cwt[AppDataConfig::kCwtCount];
};

#endif  // DEVICE_REGISTRY_H
