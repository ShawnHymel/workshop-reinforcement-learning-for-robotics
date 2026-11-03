#include <M5Unified.h>

#include "balac.h"

// Adjust left/right motor power to measure asymmetry
const float MOTOR_BOOST_LEFT = 1.0f;
const float MOTOR_BOOST_RIGHT = 0.6f;

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
  // Drive forward
  int16_t motor_left = (int16_t)(MOTOR_BOOST_LEFT * 127.0);
  int16_t motor_right = (int16_t)(MOTOR_BOOST_RIGHT * 127.0);
  Serial.printf("L: %d R: %d\n", motor_left, motor_right);
  for (int i = 0; i < 500; i++) {
    balac.SetSpeed(motor_left, motor_right);
    delay(10);
  }

  // Stop motors
  Serial.println("off");
  for (int i = 0; i < 100; i++) {
    balac.SetSpeed(0, 0);
    delay(10);
  }
}
