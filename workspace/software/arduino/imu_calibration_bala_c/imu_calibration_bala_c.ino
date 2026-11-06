/**
 * Gyro calibration for the Bala-C (M5StickC + MPU6886 via M5Unified).
 *
 * Measures the gyroscope bias (the reading when the robot is perfectly still) on all three axes,
 * checks that the robot really was still, and saves the result to flash (NVS) so the balancing
 * sketch can load it at startup.
 *
 * How to use:
 *   1. Upload this sketch and open the Serial Monitor at 115200 baud.
 *   2. Set the robot down somewhere it won't move (lying flat on a table is easiest; it only
 *      needs to be still, not flat). Let it sit ~30 s after power-up so the IMU warms up.
 *   3. Press button A (the big front button) to start. Don't touch the robot or the table.
 *   4. If the robot moved, you'll see "MOVED" and the result is not saved. Press A to retry.
 *   5. If it worked, the bias is saved to flash. Press A again any time to re-measure (e.g. to
 *      see how much the bias drifts as the robot warms up).
 *
 * The bias is reported in deg/s (raw IMU axes) and in rad/s for the two axes the balancing
 * sketch uses:
 *   pitch rate =  gyro.x  (rad/s)
 *   yaw rate   = -gyro.y  (rad/s)
 */

#include <math.h>
#include <M5Unified.h>
#include <Preferences.h>

//------------------------------------------------------------------------------
// Settings

const int WARMUP_SAMPLES = 100;         // Samples discarded after pressing A (lets you let go)
const int CAL_SAMPLES = 1000;           // Samples averaged for the bias (1000 x 5 ms = 5 s)
const int SAMPLE_DELAY_MS = 5;          // Time between samples
const float STILL_STD_DPS = 0.5f;       // Max std dev (deg/s) on any axis to count as "still"
const char* NVS_NAMESPACE = "gyrocal";  // Flash namespace (the balancing sketch must match)

//------------------------------------------------------------------------------
// Globals

Preferences prefs;

//------------------------------------------------------------------------------
// Helpers

// Print a line to both the serial monitor and the small screen
void show(const char* msg) {
  Serial.println(msg);
  M5.Display.println(msg);
}

// Clear the screen and reset the cursor
void clear_display() {
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setCursor(0, 0);
}

// Wait for button A to be pressed
void wait_for_button_a() {
  while (true) {
    M5.update();
    if (M5.BtnA.wasPressed()) {
      return;
    }
    delay(10);
  }
}

// Print whatever calibration is currently stored in flash
void print_stored() {
  prefs.begin(NVS_NAMESPACE, true);  // read-only
  bool valid = prefs.getBool("valid", false);
  if (valid) {
    Serial.printf("Stored bias (deg/s): x=%+.4f y=%+.4f z=%+.4f\n",
                  prefs.getFloat("bias_x", 0.0f),
                  prefs.getFloat("bias_y", 0.0f),
                  prefs.getFloat("bias_z", 0.0f));
  } else {
    Serial.println("No calibration stored yet.");
  }
  prefs.end();
}

//------------------------------------------------------------------------------
// Calibration

void run_calibration() {
  clear_display();
  show("Gyro cal");
  show("Hands off!");

  // Discard the first samples so the robot settles after the button press
  for (int i = 0; i < WARMUP_SAMPLES; i++) {
    M5.Imu.update();
    delay(SAMPLE_DELAY_MS);
  }

  // Accumulate sums and sums of squares (for mean and std dev) on each axis
  double sum[3] = {0.0, 0.0, 0.0};
  double sum_sq[3] = {0.0, 0.0, 0.0};
  for (int i = 0; i < CAL_SAMPLES; i++) {
    M5.Imu.update();
    auto d = M5.Imu.getImuData();
    float g[3] = {d.gyro.x, d.gyro.y, d.gyro.z};  // deg/s
    for (int a = 0; a < 3; a++) {
      sum[a] += g[a];
      sum_sq[a] += (double)g[a] * g[a];
    }
    delay(SAMPLE_DELAY_MS);
  }

  // Compute mean (the bias) and std dev (how much it wiggled) per axis
  float mean[3], stdev[3];
  bool still = true;
  for (int a = 0; a < 3; a++) {
    mean[a] = (float)(sum[a] / CAL_SAMPLES);
    double var = sum_sq[a] / CAL_SAMPLES - (double)mean[a] * mean[a];
    stdev[a] = (float)sqrt(var > 0.0 ? var : 0.0);
    if (stdev[a] > STILL_STD_DPS) {
      still = false;
    }
  }

  // Report
  Serial.println("\n=== Gyro calibration ===");
  Serial.printf("Bias    (deg/s): x=%+.4f  y=%+.4f  z=%+.4f\n", mean[0], mean[1], mean[2]);
  Serial.printf("Std dev (deg/s): x=%.4f   y=%.4f   z=%.4f\n", stdev[0], stdev[1], stdev[2]);
  Serial.printf("As used by the balancing sketch (rad/s): pitch=%+.5f  yaw=%+.5f\n",
                mean[0] * (M_PI / 180.0f), -mean[1] * (M_PI / 180.0f));

  // What this bias would do if left uncorrected
  float yaw_drift_30s = fabsf(mean[1]) * 30.0f;
  Serial.printf("Uncorrected, the heading estimate would drift ~%.1f deg in 30 s\n", yaw_drift_30s);

  clear_display();
  if (!still) {
    Serial.printf("MOVED: std dev above %.2f deg/s on at least one axis. Not saved.\n",
                  STILL_STD_DPS);
    show("MOVED!");
    show("Not saved.");
    show("Press A");
    show("to retry");
    return;
  }

  // Save to flash
  prefs.begin(NVS_NAMESPACE, false);  // read-write
  prefs.putFloat("bias_x", mean[0]);
  prefs.putFloat("bias_y", mean[1]);
  prefs.putFloat("bias_z", mean[2]);
  prefs.putBool("valid", true);
  prefs.end();

  Serial.println("Saved to flash.");
  M5.Display.printf("x %+.3f\n", mean[0]);
  M5.Display.printf("y %+.3f\n", mean[1]);
  M5.Display.printf("z %+.3f\n", mean[2]);
  show("Saved!");
  show("A: redo");
}

//------------------------------------------------------------------------------
// Main

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(0);
  M5.Display.setTextSize(1);

  Serial.begin(115200);
  delay(500);
  Serial.println("\nBala-C gyro calibration");

  if (!M5.Imu.isEnabled()) {
    Serial.println("ERROR: IMU not found");
    clear_display();
    show("No IMU!");
    while (true) {
      delay(100);
    }
  }

  print_stored();

  clear_display();
  show("Gyro cal");
  show("Keep still");
  show("Press A");
  Serial.println("Place the robot where it won't move, then press button A.");
}

void loop() {
  wait_for_button_a();
  run_calibration();
  Serial.println("Press button A to measure again.");
}
