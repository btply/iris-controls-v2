#line 1 "/home/billy/Documents/Code/iris-controls-v2/src/devices/ModbusCommissioningTable.h"
#ifndef MODBUS_COMMISSIONING_TABLE_H
#define MODBUS_COMMISSIONING_TABLE_H

#include <stddef.h>
#include <stdint.h>

namespace ModbusCommissioning {

enum class Kind : uint8_t {
  Weather = 0U,
  Cwt = 1U,
};

struct Entry {
  Kind kind;
  /** When kind == Cwt: index into DeviceRegistry::cwt[]. */
  uint8_t cwtSlotIndex;
  /** When kind == Cwt: Modbus slave ID. Ignored for Weather (fixed in WeatherDevice). */
  uint8_t slaveId;
};

/** Compile-time commissioned devices; edit this table only. */
static const Entry kEntries[] = {
    {Kind::Weather, 0U, 20U},
    {Kind::Cwt, 0U, 4U},
    {Kind::Cwt, 1U, 5U},
};

static const size_t kEntryCount = sizeof(kEntries) / sizeof(kEntries[0]);

}  // namespace ModbusCommissioning

#endif  // MODBUS_COMMISSIONING_TABLE_H
