#include <math.h>
#include <M5Unified.h>

const float COMP_ALPHA = 0.99f;
const float TIMESTEP   = 0.005f;

float pitch = 0.0f;

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.begin(115200);

  // Load IMU calibration
  // if (!M5.Imu.loadOffsetFromNVS()) {
  //   Serial.println("ERROR: No IMU calibration - run calibrate_imu first");
  //   while (1) delay(1);
  // }
}

void loop() {
  unsigned long step_start = micros();

  M5.Imu.update();
  auto d = M5.Imu.getImuData();

  // Same mapping as balance code
  float accel_fwd = d.accel.z;
  float accel_up  = -d.accel.y;
  float pitch_rate = d.gyro.x * (M_PI / 180.0f);
  float yaw_rate   = -d.gyro.y * (M_PI / 180.0f);
  float accel_pitch = atan2f(accel_fwd, accel_up);

  pitch = COMP_ALPHA * (pitch + pitch_rate * TIMESTEP)
        + (1.0f - COMP_ALPHA) * accel_pitch;

  static int n = 0;
  if (++n >= 20) {                 // print ~every 100ms
    n = 0;
    Serial.printf("pitch=%+6.1f deg | pitch_rate=%+6.2f | yaw_rate=%+6.2f | accelY=%+5.2f accelZ=%+5.2f\n",
                  pitch * 180.0f / M_PI, pitch_rate, yaw_rate, accel_fwd, accel_up);
  }

  while (micros() - step_start < (unsigned long)(TIMESTEP * 1e6f));
}