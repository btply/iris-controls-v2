#line 1 "/home/billy/Documents/Code/iris-controls-v2/src/config/CurtainHardware.h"
#ifndef CURTAIN_HARDWARE_H
#define CURTAIN_HARDWARE_H

#include <Arduino.h>
#include <stdint.h>

namespace CurtainHardware {

static const uint8_t kCurtainCount = 4U;
static const uint8_t kOutputChannelCount = 4U;

static const uint8_t kDirectionPin = D0;
static const uint8_t kDirectionLedPin = LED_D0;
static const bool kDirectionHighIsOpen = true;

static const uint8_t kOutput1Pin = D1;
static const uint8_t kOutput1LedPin = LED_D1;
static const uint8_t kOutput2Pin = D2;
static const uint8_t kOutput2LedPin = LED_D2;
static const uint8_t kOutput3Pin = D3;
static const uint8_t kOutput3LedPin = LED_D3;
static const uint8_t kOutput4ExpansionChannel = 0U;

static inline bool mapCurtainIdToOutputChannel(uint8_t curtainId, uint8_t* channelOut) {
  if (channelOut == nullptr || curtainId >= kCurtainCount) {
    return false;
  }
  *channelOut = curtainId;
  return true;
}

}  // namespace CurtainHardware

#endif  // CURTAIN_HARDWARE_H
