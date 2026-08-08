#include <math.h>
#include <M5Unified.h>

#include "balac.h"

// Constants
const float COMP_ALPHA = 0.99f;         // Alpha for complementary filter (must match training)
const float VEL_LEAK = 0.995;           // Integrator leak for estimating velocity from commands
const float POS_LEAK = 0.995;           // Integrator leak for estimating position from velocity commands estimate
const float HEADING_LEAK = 1.0;         // Integrator leak for estimating heading from yaw rate
const float TIMESTEP = 0.005f;          // Time (sec) between intervals
const float TIP_THRESHOLD = 0.79f;      // radians (~45 deg), stop motors if exceeded
const unsigned long RESET_TIME_MS = 1000; // How long to wait before running again

// Globals
BalaC balac;
float pitch = 0.0f;
float heading_est = 0.0f; 
bool tipped = false;
unsigned long start_time = 0;

void setup() {
  // Initialize M5 library/drivers
  auto cfg = M5.config();
  M5.begin(cfg);

  // Initialize BalaC library
  balac.begin();

  // Initialize serial
  Serial.begin(115200);
  Serial.println("Balance bot");

  // Stop motors
  balac.SetSpeed(0, 0);

  // Wait
  Serial.printf("Wait %lu sec to start test...\n", RESET_TIME_MS / 1000);
  delay(RESET_TIME_MS);
  start_time = millis();
}

void loop() {
  
  // Get timestamp for pacing to timestep interval
  unsigned long step_start = micros();

  // Read IMU (+Y is down toward ground, +Z is forward)
  M5.Imu.update();
  auto imu_data = M5.Imu.getImuData();
  float accel_fwd = imu_data.accel.z;
  float accel_up = -imu_data.accel.y;
  float pitch_rate = imu_data.gyro.x * (M_PI / 180.0f); 
  float yaw_rate = -imu_data.gyro.y * (M_PI / 180.0f);

  // Calculate the pitch from the IMU (note: motion can make this inaccurate)
  float accel_pitch = atan2f(accel_fwd, accel_up);

  // ...so we use a complementary filter to combine the accel and gyro readings to estimate pitch
  pitch = COMP_ALPHA * (pitch + pitch_rate * TIMESTEP) + (1.0f - COMP_ALPHA) * accel_pitch;

  // Estimate heading using a leaky integrator with yaw rate
  heading_est = HEADING_LEAK * (heading_est + (yaw_rate * TIMESTEP));

  // Check if tipped
  if (fabsf(pitch) > TIP_THRESHOLD) {
    tipped = true;
  }

  // Print if not tipped
  if (!tipped) {
    Serial.printf("t=%lu pitch=%.4f rate=%.4f\n", millis() - start_time, pitch, pitch_rate);
  } else {
    // Wait for someone to pick the bot up
    if (fabsf(pitch) <= 0.3) {
      tipped = false;

      // Reset the tip sensor pitch and filters
      pitch = 0.0f;

      // reset the accumulators
      heading_est = 0.0f;
      yaw_rate = 0.0f;

      // Wait a moment before starting
      Serial.printf("Untipped! Starting in %lu seconds\n", RESET_TIME_MS / 1000);
      delay(RESET_TIME_MS);
      Serial.printf("Running...\n");
      start_time = millis();
    }
  }

  // Pace to TIMESTEP
  while (micros() - step_start < (unsigned long)(TIMESTEP * 1e6f));
}
