#include <Arduino.h>
#line 1 "/home/billy/Documents/Code/iris-controls-v2/iris-controls-v2.ino"
#include "src/core/AppLifecycle.h"

AppLifecycle appLifecycle;

#line 5 "/home/billy/Documents/Code/iris-controls-v2/iris-controls-v2.ino"
void setup();
#line 9 "/home/billy/Documents/Code/iris-controls-v2/iris-controls-v2.ino"
void loop();
#line 5 "/home/billy/Documents/Code/iris-controls-v2/iris-controls-v2.ino"
void setup() {
  appLifecycle.begin();
}

void loop() {
  appLifecycle.tick();
}

