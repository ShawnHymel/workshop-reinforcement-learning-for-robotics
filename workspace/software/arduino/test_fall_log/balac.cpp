/**
 * BalaC-Plus motor interface
 *
 * See balac.h for details.
 */

#include "balac.h"
#include <Wire.h>

// BalaC-Plus STM32 I2C address and registers.
// Each motor is its own single-byte register; there is no multi-byte
// motor write like the Bala 2 uses.
static const uint8_t BALAC_ADDR = 0x38;
static const uint8_t REG_MOTOR_LEFT = 0x00;
static const uint8_t REG_MOTOR_RIGHT = 0x01;

// The STM32 takes a signed 8-bit speed. We clamp to +/-127 (rather than -128)
// to keep the forward and reverse ranges symmetric.
static const int16_t SPEED_MAX = 127;
static const int16_t SPEED_MIN = -127;

// Sign convention, measured on hardware: a positive byte on the wire spins the
// LEFT motor backward and the RIGHT motor forward. Inverting the left channel
// gives us "positive = forward" on both wheels at the API level.
//
// Note this is the opposite of what M5Stack's BalaCplus.ino does, which sends
// +power to the left and -power to the right. Their "positive power" therefore
// means backward. See the porting note about negating the PID output.
static const bool INVERT_LEFT_MOTOR = true;
static const bool INVERT_RIGHT_MOTOR = false;

// ---------------------------------------------------------------------------
// BalaC class

BalaC::BalaC() {
  i2c_mutex = NULL;
}

void BalaC::begin(int sda, int scl) {
  Wire.begin(sda, scl);
}

void BalaC::WriteMotor(uint8_t reg, int8_t speed) {
  Wire.beginTransmission(BALAC_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)speed);
  Wire.endTransmission();
}

void BalaC::SetSpeed(int16_t wheel_left, int16_t wheel_right) {
  // Clamp first, then invert, so that negating can never overflow
  int8_t left = (int8_t)constrain(wheel_left, SPEED_MIN, SPEED_MAX);
  int8_t right = (int8_t)constrain(wheel_right, SPEED_MIN, SPEED_MAX);

  if (INVERT_LEFT_MOTOR) {
    left = -left;
  }
  if (INVERT_RIGHT_MOTOR) {
    right = -right;
  }

  // Hold the lock across both writes so the two wheels always update together
  if (i2c_mutex != NULL) { xSemaphoreTake(i2c_mutex, portMAX_DELAY); }
  WriteMotor(REG_MOTOR_LEFT, left);
  WriteMotor(REG_MOTOR_RIGHT, right);
  if (i2c_mutex != NULL) { xSemaphoreGive(i2c_mutex); }
}

void BalaC::Stop() {
  SetSpeed(0, 0);
}

void BalaC::SetMutex(SemaphoreHandle_t mutex) {
  i2c_mutex = mutex;
}