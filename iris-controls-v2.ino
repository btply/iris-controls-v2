#include "src/core/AppLifecycle.h"

AppLifecycle appLifecycle;

void setup() {
  appLifecycle.begin();
}

void loop() {
  appLifecycle.tick();
}
