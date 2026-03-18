#line 1 "/home/billy/Documents/Code/iris-controls-v2/src/services/LoggerService.h"
#ifndef LOGGER_SERVICE_H
#define LOGGER_SERVICE_H

#include <Arduino.h>
#include <stdarg.h>
#include <stddef.h>

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

 private:
  static const char* levelTag(Level level);
  static void writeLine(Level level, const char* module, const char* message);

  static void (*timeFormatter)(char* buffer, size_t bufferSize);
};

#endif  // LOGGER_SERVICE_H
