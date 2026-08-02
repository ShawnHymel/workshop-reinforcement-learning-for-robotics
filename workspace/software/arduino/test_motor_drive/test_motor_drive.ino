#include <M5Unified.h>

#include "balac.h"

// Globals
BalaC balac;

void setup() {
  // Initialize M5 library/drivers
  auto cfg = M5.config();
  M5.begin(cfg);

  balac.begin();

  // Initialize serial
  Serial.begin(115200);
  Serial.println("BalaC Test");

  // Stop motors
  balac.SetSpeed(0, 0);
}

void loop() {
  Serial.print("on...");
  for (int i = 0; i < 300; i++) {
    balac.SetSpeed(-127, -127);
    delay(10);
  }
  Serial.println("off");
  for (int i = 0; i < 1500; i++) {
    balac.SetSpeed(0, 0);
    delay(10);
  }
}
