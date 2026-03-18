#line 1 "/home/billy/Documents/Code/iris-controls-v2/src/services/LoggerService.h"
#ifndef LOGGER_SERVICE_H
#define LOGGER_SERVICE_H

#include <Arduino.h>
#include <stdarg.h>
#include <stddef.h>
#include <mbed.h>

class LoggerService {
 public:
  enum class Level : uint8_t {
    Info = 0,
    Warn = 1,
    Error = 2,
  };

  static void begin(unsigned long baud, unsigned long waitMs);
  static void setTimeFormatter(void (*formatter)(char* buffer, size_t bufferSize));

  static void info(const char* module, const char* message);
  static void warn(const char* module, const char* message);
  static void error(const char* module, const char* message);
  static void printf(Level level, const char* module, const char* fmt, ...);
  static void vprintf(Level level, const char* module, const char* fmt, va_list args);

  /** Non-blocking: for use from worker threads. Drops newest if ring full. */
  static void enqueue(Level level, const char* module, const char* message);
  static void enqueuePrintf(Level level, const char* module, const char* fmt, ...);
  /** Call from main thread only; flushes enqueued lines to Serial. */
  static void drain();

 private:
  static const char* levelTag(Level level);
  static void writeLine(Level level, const char* module, const char* message);

  static const size_t kLogRingSize = 8U;
  static const size_t kLogModuleLen = 24U;
  static const size_t kLogMessageLen = 128U;

  struct LogEntry {
    Level level;
    char module[kLogModuleLen];
    char message[kLogMessageLen];
  };

  static void (*timeFormatter)(char* buffer, size_t bufferSize);
  static LogEntry logRing[kLogRingSize];
  static size_t logRingHead;
  static size_t logRingTail;
  static rtos::Mutex logRingMutex;
};

#endif  // LOGGER_SERVICE_H
