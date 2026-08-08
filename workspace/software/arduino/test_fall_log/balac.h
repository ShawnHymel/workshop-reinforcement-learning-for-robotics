/**
 * BalaC-Plus motor interface
 *
 * Minimal I2C driver for the STM32F030F4P6 motor controller on the M5Stack
 * BalaC-Plus balancing robot.
 *
 * Protocol derived from M5Stack's BalaCplus.ino example:
 *   https://github.com/m5stack/M5-ProductExampleCodes/blob/master/App/BalaC-Plus/Arduino/BalaCplus/BalaCplus.ino
 *
 * NOTE: Unlike the Bala 2 base, the BalaC-Plus has no wheel encoders and no
 * servo headers, so this class exposes motor control only.
 *
 * Based on the Bala 2 interface by M5Stack, modified by Shawn Hymel
 */

#ifndef _BALAC_H__
#define _BALAC_H__

#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class BalaC {
 public:
  BalaC();

  // Initialize the I2C bus. Defaults match the BalaC-Plus HAT pinout.
  // Skip this if your sketch already calls Wire.begin() itself.
  void begin(int sda = 0, int scl = 26);

  // -127 ~ 127. Positive values drive both wheels forward; the right motor's
  // mechanical inversion is handled internally. Values outside the range are
  // clamped, not wrapped.
  void SetSpeed(int16_t wheel_left, int16_t wheel_right);

  // Convenience for SetSpeed(0, 0)
  void Stop();

  // Optional I2C lock, for sharing the bus with another FreeRTOS task
  void SetMutex(SemaphoreHandle_t mutex);

 private:
  void WriteMotor(uint8_t reg, int8_t speed);

  SemaphoreHandle_t i2c_mutex;
};

#endif  // _BALAC_H__