#include "LoggerService.h"

#include <stdio.h>

void (*LoggerService::timeFormatter)(char* buffer, size_t bufferSize) = nullptr;

void LoggerService::begin(unsigned long baud, unsigned long waitMs) {
  Serial.begin(baud);
  const unsigned long startMs = millis();
  while (!Serial && (millis() - startMs) < waitMs) {
  }
}

void LoggerService::setTimeFormatter(void (*formatter)(char* buffer, size_t bufferSize)) {
  timeFormatter = formatter;
}

void LoggerService::info(const char* module, const char* message) {
  writeLine(Level::Info, module, message);
}

void LoggerService::warn(const char* module, const char* message) {
  writeLine(Level::Warn, module, message);
}

void LoggerService::error(const char* module, const char* message) {
  writeLine(Level::Error, module, message);
}

void LoggerService::printf(Level level, const char* module, const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vprintf(level, module, fmt, args);
  va_end(args);
}

void LoggerService::vprintf(Level level,
                            const char* module,
                            const char* fmt,
                            va_list args) {
  char message[256];
  const int written = vsnprintf(message, sizeof(message), fmt, args);
  if (written < 0) {
    writeLine(Level::Error, "LoggerService", "format_error");
    return;
  }
  message[sizeof(message) - 1] = '\0';
  writeLine(level, module, message);
}

const char* LoggerService::levelTag(Level level) {
  switch (level) {
    case Level::Info:
      return "INFO";
    case Level::Warn:
      return "WARN";
    case Level::Error:
      return "ERROR";
    default:
      return "INFO";
  }
}

void LoggerService::writeLine(Level level, const char* module, const char* message) {
  char timestamp[24];
  if (timeFormatter != nullptr) {
    timeFormatter(timestamp, sizeof(timestamp));
  } else {
    snprintf(timestamp, sizeof(timestamp), "t+%lu", millis());
  }
  timestamp[sizeof(timestamp) - 1] = '\0';

  char line[360];
  snprintf(line,
           sizeof(line),
           "[%s] %s [%s] %s",
           timestamp,
           levelTag(level),
           module != nullptr ? module : "core",
           message != nullptr ? message : "");
  line[sizeof(line) - 1] = '\0';
  if (Serial) {
    Serial.println(line);
  }
}
