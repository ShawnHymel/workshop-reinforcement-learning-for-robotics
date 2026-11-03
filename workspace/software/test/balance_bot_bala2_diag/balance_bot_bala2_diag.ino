/**
 * Balance bot firmware with hardware diagnostics for sim-to-real tuning.
 *
 * Select a mode with TEST_MODE below, then open the Serial Monitor at 921600 baud.
 *
 *   TEST_MODE 0: Normal balancing (same behavior as before) plus loop timing statistics,
 *                printed once per second while the robot is balancing.
 *                -> Tells us whether the 5 ms loop ever overruns and how long each part takes.
 *
 *   TEST_MODE 1: Motor step response test. HOLD THE ROBOT SO BOTH WHEELS SPIN FREELY.
 *                Steps the motors from 0 to several commands (both directions), logs the
 *                encoders every 5 ms, and reports dead time, rise time, stop time, and
 *                steady-state speed for each wheel.
 *                -> Gives us the real action delay, motor lag, and speed-vs-command gain.
 *
 *   TEST_MODE 2: Motor deadband test. HOLD THE ROBOT SO BOTH WHEELS SPIN FREELY.
 *                Slowly ramps the command up from 0 and back down (both directions) and
 *                reports the command at which each wheel starts and stops turning.
 *                -> Gives us the motor deadband / stiction.
 *
 * In modes 1 and 2, press button A to start (and press again to re-run).
 *
 * Note: battery voltage changes motor results, so test with a reasonably charged battery and
 * note the voltage printed with each test.
 */

#include <math.h>
#include <M5Unified.h>

#include "actor.h"
#include "bala.h"

//------------------------------------------------------------------------------
// Test selection

#define TEST_MODE 1           // 0 = balance + timing, 1 = motor step test, 2 = deadband test
#define DUMP_STEP_CSV 0       // Mode 1: also print the raw 5 ms encoder log as CSV lines

//------------------------------------------------------------------------------
// Settings (unchanged from the balancing sketch)

const float PITCH_OFFSET = 0.0f;        // Tune this so the robot stays upright (+: back bias, -: front bias)
const float MOTOR_BOOST = 1.0f;         // Tune this so the motors are responsive on battery power
const float ACTION_DEADBAND = 0.0f;     // Tune this: Ignore small motor corrections
const float ACTION_ALPHA = 0.0f;        // Tune this: alpha for low-pass filter (higher: smoother, more lag)
const float COMP_ALPHA = 0.99f;         // Alpha for complementary filter (must match training)
const float TIMESTEP = 0.005f;          // Time (sec) between intervals
const float MOTOR_SCALE = 1023.0f;      // Scale motors from [-1, 1] to [-1023, 1023]
const float ENC_TICKS_PER_REV = 420.0f; // Encoder ticks per wheel revolution
const float TIP_THRESHOLD = 0.79f;      // radians (~45 deg), stop motors if exceeded
const int16_t MOTOR_DIR_LEFT = -1;      // Left motor direction
const int16_t MOTOR_DIR_RIGHT = -1;     // Right motor direction
const int32_t ENC_DIR_LEFT = 1;         // Left encoder direction
const int32_t ENC_DIR_RIGHT = 1;        // Right encoder direction
const unsigned long RESET_TIME_MS = 1000; // How long to wait before running again

// Derived constants
const float ENC_TICKS_TO_RADS = (2.0f * M_PI) / ENC_TICKS_PER_REV;
const uint32_t TIMESTEP_US = (uint32_t)(TIMESTEP * 1e6f);

//------------------------------------------------------------------------------
// Diagnostic settings

// Mode 0: timing
const int TIMING_PRINT_STEPS = 200;     // Print a timing summary every N balancing steps (1 s)
const uint32_t LATE_SLACK_US = 250;     // A loop period counts as "late" if > TIMESTEP + this

// Mode 1: step test
const float STEP_CMDS[] = {0.15f, 0.3f, 0.6f, 1.0f, -0.15f, -0.3f, -0.6f, -1.0f};
const int NUM_STEP_CMDS = sizeof(STEP_CMDS) / sizeof(STEP_CMDS[0]);
const int STEP_PRE = 100;               // Steps at 0 before the step (0.5 s)
const int STEP_ON = 300;                // Steps at the commanded value (1.5 s)
const int STEP_OFF = 300;               // Steps back at 0 to watch it stop (1.5 s)
const int STEP_TOTAL = STEP_PRE + STEP_ON + STEP_OFF;
const int STEADY_WINDOW = 100;          // Last N "on" steps used to measure steady speed (0.5 s)

// Mode 2: deadband ramp
const float RAMP_MAX = 0.4f;            // Ramp command from 0 up to this and back down
const int RAMP_STEPS = 1600;            // Steps per ramp direction (8 s up, 8 s down)
const int MOVE_WINDOW = 20;             // Wheel is "moving" if it turned >= MOVE_TICKS in this many steps (100 ms)
const int32_t MOVE_TICKS = 2;

//------------------------------------------------------------------------------
// Types
//
// Note: these must be defined before any function. The Arduino IDE inserts auto-generated
// function prototypes near the top of the sketch, so a type used in a function signature has
// to be declared above all functions or compilation fails ("does not name a type").

// Mode 1: results for one wheel in one step trial
struct StepResult {
  float steady_rads;    // Steady-state wheel speed during the step (rad/s)
  float first_tick_ms;  // Time from command change to the first encoder tick
  float rise63_ms;      // Time from command change to 63% of steady speed
  float stop37_ms;      // Time from command back to 0 until speed falls to 37% of steady
};

//------------------------------------------------------------------------------
// Globals

Bala bala;
float pitch = 0.0f;
int32_t prev_enc_left = 0;
int32_t prev_enc_right = 0;
bool tipped = false;
float action_filtered[2] = {0.0f, 0.0f};

//------------------------------------------------------------------------------
// Shared helpers

// Read encoders with direction correction applied
void read_encoders(int32_t* left, int32_t* right) {
  bala.GetEncoder(left, right);
  *left  = ENC_DIR_LEFT  * (*left);
  *right = ENC_DIR_RIGHT * (*right);
}

// Send normalized [-1, 1] motor commands, using the same scaling and direction as balancing
void set_motors(float left, float right) {
  left  = constrain(left,  -1.0f, 1.0f);
  right = constrain(right, -1.0f, 1.0f);
  int16_t motor_left  = MOTOR_DIR_LEFT  * (int16_t)(left  * MOTOR_SCALE * MOTOR_BOOST);
  int16_t motor_right = MOTOR_DIR_RIGHT * (int16_t)(right * MOTOR_SCALE * MOTOR_BOOST);
  bala.SetSpeed(motor_left, motor_right);
}

// Busy-wait until micros() reaches t_us (handles timer wraparound)
void wait_until(uint32_t t_us) {
  while ((int32_t)(micros() - t_us) < 0) {
  }
}

// Wait for button A to be pressed
void wait_for_button_a() {
  Serial.println("Press button A to start...");
  while (true) {
    M5.update();
    if (M5.BtnA.wasPressed()) {
      break;
    }
    delay(10);
  }
  delay(500);  // Give your hand time to settle after pressing
}

//------------------------------------------------------------------------------
// Mode 0: Loop timing statistics

#if TEST_MODE == 0

struct TimingStat {
  uint32_t min_us;
  uint32_t max_us;
  uint64_t sum_us;
  uint32_t count;

  void reset() {
    min_us = UINT32_MAX;
    max_us = 0;
    sum_us = 0;
    count = 0;
  }

  void add(uint32_t v) {
    if (v < min_us) min_us = v;
    if (v > max_us) max_us = v;
    sum_us += v;
    count++;
  }

  unsigned long avg() const {
    return count ? (unsigned long)(sum_us / count) : 0;
  }
};

TimingStat t_imu;     // IMU read
TimingStat t_enc;     // Encoder read (I2C to STM32)
TimingStat t_nn;      // Actor forward pass
TimingStat t_motor;   // Action post-processing + motor write (I2C to STM32)
TimingStat t_work;    // Everything from loop start to just before the pacing wait
TimingStat t_period;  // Actual time between the starts of consecutive loops

uint32_t n_overrun = 0;       // Steps where work took longer than one timestep (this interval)
uint32_t n_late = 0;          // Steps where the actual period was late (this interval)
uint32_t total_overrun = 0;   // Since boot
uint32_t total_steps = 0;     // Since boot

unsigned long prev_step_start = 0;
bool have_prev_step = false;  // False right after a pause, so pauses aren't counted as periods

void reset_timing_stats() {
  t_imu.reset();
  t_enc.reset();
  t_nn.reset();
  t_motor.reset();
  t_work.reset();
  t_period.reset();
  n_overrun = 0;
  n_late = 0;
}

void print_timing_stats() {
  Serial.printf(
    "[timing us] period avg=%lu max=%lu late=%lu | work avg=%lu max=%lu overrun=%lu | "
    "imu %lu/%lu  enc %lu/%lu  nn %lu/%lu  motor %lu/%lu (avg/max) | n=%lu total_overrun=%lu/%lu\n",
    t_period.avg(), (unsigned long)t_period.max_us, (unsigned long)n_late,
    t_work.avg(), (unsigned long)t_work.max_us, (unsigned long)n_overrun,
    t_imu.avg(), (unsigned long)t_imu.max_us,
    t_enc.avg(), (unsigned long)t_enc.max_us,
    t_nn.avg(), (unsigned long)t_nn.max_us,
    t_motor.avg(), (unsigned long)t_motor.max_us,
    (unsigned long)t_work.count, (unsigned long)total_overrun, (unsigned long)total_steps);
}

#endif  // TEST_MODE == 0

//------------------------------------------------------------------------------
// Mode 1: Motor step response test

#if TEST_MODE == 1

static uint32_t log_t_us[STEP_TOTAL];
static int32_t log_enc_l[STEP_TOTAL];
static int32_t log_enc_r[STEP_TOTAL];

StepResult results_l[NUM_STEP_CMDS];
StepResult results_r[NUM_STEP_CMDS];

// Wheel speed at step k from a centered 4-step window (rad/s)
float vel_at(const int32_t* enc, int k) {
  return (float)(enc[k + 2] - enc[k - 2]) * ENC_TICKS_TO_RADS / (4.0f * TIMESTEP);
}

// Milliseconds between two logged steps
float ms_between(int k_from, int k_to) {
  return (float)(log_t_us[k_to] - log_t_us[k_from]) / 1000.0f;
}

StepResult analyze_wheel(const int32_t* enc) {
  StepResult res = {0.0f, -1.0f, -1.0f, -1.0f};
  const int k_on = STEP_PRE;
  const int k_off = STEP_PRE + STEP_ON;

  // Steady speed over the last part of the "on" phase
  res.steady_rads = (float)(enc[k_off - 1] - enc[k_off - 1 - STEADY_WINDOW]) *
                    ENC_TICKS_TO_RADS / ((float)STEADY_WINDOW * TIMESTEP);
  float mag = fabsf(res.steady_rads);
  float sgn = (res.steady_rads >= 0.0f) ? 1.0f : -1.0f;
  if (mag < 0.5f) {
    return res;  // Wheel didn't really move: probably inside the deadband
  }

  // Dead time: first encoder change after the command changes
  for (int k = k_on + 1; k < k_off; k++) {
    if (enc[k] != enc[k_on]) {
      res.first_tick_ms = ms_between(k_on, k);
      break;
    }
  }

  // Rise time: first time speed reaches 63% of steady
  for (int k = k_on + 2; k < k_off - 2; k++) {
    if (sgn * vel_at(enc, k) >= 0.63f * mag) {
      res.rise63_ms = ms_between(k_on, k);
      break;
    }
  }

  // Stop time: first time speed falls to 37% of steady after the command returns to 0
  for (int k = k_off + 2; k < STEP_TOTAL - 2; k++) {
    if (sgn * vel_at(enc, k) <= 0.37f * mag) {
      res.stop37_ms = ms_between(k_off, k);
      break;
    }
  }

  return res;
}

void print_result(const char* name, float cmd, const StepResult& res) {
  float gain = (fabsf(cmd) > 0.0f) ? res.steady_rads / cmd : 0.0f;
  Serial.printf("  %-5s steady=%+7.2f rad/s (%5.1f rad/s per unit cmd)  first tick=%6.1f ms  "
                "63%% rise=%6.1f ms  37%% stop=%6.1f ms\n",
                name, res.steady_rads, gain, res.first_tick_ms, res.rise63_ms, res.stop37_ms);
}

void run_step_trial(int trial, float cmd) {
  int32_t enc_l, enc_r;

  // Let the wheels come fully to rest
  set_motors(0.0f, 0.0f);
  delay(1000);

  // Run the trial at the same 5 ms rate and order as the balancing loop:
  // read encoders, then write the motor command
  uint32_t next_us = micros();
  for (int k = 0; k < STEP_TOTAL; k++) {
    wait_until(next_us);
    next_us += TIMESTEP_US;

    log_t_us[k] = micros();
    read_encoders(&enc_l, &enc_r);
    log_enc_l[k] = enc_l;
    log_enc_r[k] = enc_r;

    float c = (k >= STEP_PRE && k < STEP_PRE + STEP_ON) ? cmd : 0.0f;
    set_motors(c, c);
  }
  set_motors(0.0f, 0.0f);

  // Analyze and report
  results_l[trial] = analyze_wheel(log_enc_l);
  results_r[trial] = analyze_wheel(log_enc_r);
  Serial.printf("\nTrial %d: cmd=%+.2f  batt=%dmV  avg period=%.3f ms\n",
                trial, cmd, (int)M5.Power.getBatteryVoltage(),
                ms_between(0, STEP_TOTAL - 1) / (float)(STEP_TOTAL - 1));
  print_result("left", cmd, results_l[trial]);
  print_result("right", cmd, results_r[trial]);

#if DUMP_STEP_CSV
  // Raw log, one line per 5 ms step. Lines start with "csv," so they're easy to filter.
  // t_ms is relative to the moment the command changed.
  Serial.println("csv,trial,k,t_ms,cmd,enc_l,enc_r");
  for (int k = 0; k < STEP_TOTAL; k++) {
    float c = (k >= STEP_PRE && k < STEP_PRE + STEP_ON) ? cmd : 0.0f;
    Serial.printf("csv,%d,%d,%.3f,%.2f,%ld,%ld\n",
                  trial, k, (float)((int32_t)(log_t_us[k] - log_t_us[STEP_PRE])) / 1000.0f,
                  c, (long)log_enc_l[k], (long)log_enc_r[k]);
  }
#endif
}

void run_step_tests() {
  Serial.println("\n=== Motor step response test ===");
  Serial.println("Hold the robot so BOTH wheels spin freely (not touching anything).");
  wait_for_button_a();

  for (int i = 0; i < NUM_STEP_CMDS; i++) {
    run_step_trial(i, STEP_CMDS[i]);
  }

  // Summary table
  Serial.println("\n=== Step test summary ===");
  Serial.println("cmd    | L steady  R steady (rad/s) | L tick  R tick | L rise63  R rise63 | L stop37  R stop37 (ms)");
  for (int i = 0; i < NUM_STEP_CMDS; i++) {
    Serial.printf("%+.2f  | %+8.2f  %+8.2f        | %6.1f  %6.1f | %8.1f  %8.1f | %8.1f  %8.1f\n",
                  STEP_CMDS[i],
                  results_l[i].steady_rads, results_r[i].steady_rads,
                  results_l[i].first_tick_ms, results_r[i].first_tick_ms,
                  results_l[i].rise63_ms, results_r[i].rise63_ms,
                  results_l[i].stop37_ms, results_r[i].stop37_ms);
  }
  Serial.println("(-1.0 means not detected, e.g. the wheel didn't move at that command)");
  Serial.println("Done. Press button A to run again.");
}

#endif  // TEST_MODE == 1

//------------------------------------------------------------------------------
// Mode 2: Motor deadband ramp test

#if TEST_MODE == 2

// Tracks whether one wheel is moving, using encoder history over MOVE_WINDOW steps
struct MoveDetector {
  int32_t hist[MOVE_WINDOW + 1];
  int idx;
  int filled;

  void reset() {
    idx = 0;
    filled = 0;
  }

  // Add a new encoder reading; returns true if the wheel moved >= MOVE_TICKS in the window
  bool update(int32_t enc) {
    hist[idx] = enc;
    int oldest = (idx + 1) % (MOVE_WINDOW + 1);
    idx = oldest;
    if (filled < MOVE_WINDOW + 1) {
      filled++;
      if (filled < MOVE_WINDOW + 1) {
        return false;
      }
    }
    return labs((long)(enc - hist[oldest])) >= MOVE_TICKS;
  }
};

// Command at ramp step k (up phase: 0 -> max, down phase: max -> 0)
float ramp_cmd(float dir, bool up, int k) {
  if (k < 0) k = 0;
  float frac = (float)k / (float)(RAMP_STEPS - 1);
  return dir * RAMP_MAX * (up ? frac : (1.0f - frac));
}

void run_ramp(float dir) {
  MoveDetector det_l, det_r;
  int32_t enc_l, enc_r;
  float start_l = -1.0f, start_r = -1.0f;   // Command where each wheel started turning
  float stop_l = -1.0f, stop_r = -1.0f;     // Command where each wheel stopped turning
  bool moving_l = false, moving_r = false;

  set_motors(0.0f, 0.0f);
  delay(1000);
  det_l.reset();
  det_r.reset();

  // Movement is detected up to MOVE_WINDOW steps after it begins, so report the command from
  // half a window earlier as the best estimate
  const int lag = MOVE_WINDOW / 2;

  for (int phase = 0; phase < 2; phase++) {
    bool up = (phase == 0);
    uint32_t next_us = micros();

    for (int k = 0; k < RAMP_STEPS; k++) {
      wait_until(next_us);
      next_us += TIMESTEP_US;

      read_encoders(&enc_l, &enc_r);
      bool now_l = det_l.update(enc_l);
      bool now_r = det_r.update(enc_r);

      if (up) {
        if (now_l && start_l < 0.0f) start_l = fabsf(ramp_cmd(dir, true, k - lag));
        if (now_r && start_r < 0.0f) start_r = fabsf(ramp_cmd(dir, true, k - lag));
      } else {
        if (moving_l && !now_l && stop_l < 0.0f) stop_l = fabsf(ramp_cmd(dir, false, k - lag));
        if (moving_r && !now_r && stop_r < 0.0f) stop_r = fabsf(ramp_cmd(dir, false, k - lag));
      }
      moving_l = now_l;
      moving_r = now_r;

      float c = ramp_cmd(dir, up, k);
      set_motors(c, c);
    }
  }
  set_motors(0.0f, 0.0f);

  Serial.printf("\nDirection %s  batt=%dmV\n", (dir > 0.0f) ? "+" : "-",
                (int)M5.Power.getBatteryVoltage());
  Serial.printf("  left : starts turning at |cmd|=%.3f, stops at |cmd|=%.3f\n", start_l, stop_l);
  Serial.printf("  right: starts turning at |cmd|=%.3f, stops at |cmd|=%.3f\n", start_r, stop_r);
}

void run_deadband_tests() {
  Serial.println("\n=== Motor deadband test ===");
  Serial.println("Hold the robot so BOTH wheels spin freely (not touching anything).");
  Serial.printf("Each direction ramps 0 -> %.2f -> 0 over %.0f s.\n",
                RAMP_MAX, 2.0f * RAMP_STEPS * TIMESTEP);
  wait_for_button_a();

  run_ramp(1.0f);
  run_ramp(-1.0f);

  Serial.println("\n(-1.000 means not detected. If 'starts' is -1, raise RAMP_MAX.)");
  Serial.println("Resolution is about +/-0.005 in command.");
  Serial.println("Done. Press button A to run again.");
}

#endif  // TEST_MODE == 2

//------------------------------------------------------------------------------
// Setup

void setup() {
  bool m5_ret;

  // Initialize M5 library/drivers
  auto cfg = M5.config();
  M5.begin(cfg);

  // Initialize serial: fast baud and a large TX buffer so prints don't stall the control loop
  Serial.setTxBufferSize(4096);
  Serial.begin(921600);
  Serial.println("Balance bot (diagnostic build)");
  Serial.printf("TEST_MODE = %d\n", TEST_MODE);

  // Stop motors
  bala.SetSpeed(0, 0);

#if TEST_MODE == 0
  // Load calibration data from NVS
  m5_ret = M5.Imu.loadOffsetFromNVS();
  if (!m5_ret) {
    Serial.println("ERROR: No IMU calibration found!");
    Serial.println("Run calibrate_imu.ino first");
    while (1) {
      delay(1);
    }
  }
  Serial.println("IMU calibration loaded");

  // Zero out encoders
  bala.ClearEncoder();

  // Prepare timing stats
  reset_timing_stats();

  // Wait to start
  Serial.printf("Starting in %lu seconds, place robot upright\n", RESET_TIME_MS / 1000);
  delay(RESET_TIME_MS);
  Serial.println("Running...");
#else
  (void)m5_ret;
  bala.ClearEncoder();
#endif
}

//------------------------------------------------------------------------------
// Loop

#if TEST_MODE == 0

void loop() {
  int32_t enc_left;
  int32_t enc_right;
  static int print_counter = 0;

  // Get timestamp for pacing to timestep interval
  unsigned long step_start = micros();

  // Measure the actual loop period (skipped right after pauses)
  if (have_prev_step && !tipped) {
    uint32_t period = (uint32_t)(step_start - prev_step_start);
    t_period.add(period);
    if (period > TIMESTEP_US + LATE_SLACK_US) {
      n_late++;
    }
  }
  prev_step_start = step_start;
  have_prev_step = true;

  // Read IMU
  uint32_t t0 = micros();
  M5.Imu.update();
  auto imu_data = M5.Imu.getImuData();
  uint32_t t1 = micros();
  float accel_fwd  = imu_data.accel.y;
  float accel_up   = imu_data.accel.z;

  // Use complementary filter to calculate pitch
  float pitch_rate = -imu_data.gyro.x * (M_PI / 180.0f);
  float accel_pitch = -atan2f(accel_fwd, accel_up);
  pitch = COMP_ALPHA * (pitch + (pitch_rate * TIMESTEP)) + ((1.0f - COMP_ALPHA) * accel_pitch);

  // Read encoders (flip sign if needed)
  uint32_t t2 = micros();
  bala.GetEncoder(&enc_left, &enc_right);
  uint32_t t3 = micros();
  enc_left = ENC_DIR_LEFT * enc_left;
  enc_right = ENC_DIR_RIGHT * enc_right;

  // Compute wheel velocities
  int32_t delta_enc_left  = enc_left  - prev_enc_left;
  int32_t delta_enc_right = enc_right - prev_enc_right;
  prev_enc_left  = enc_left;
  prev_enc_right = enc_right;
  float wheel_vel_left  = (float)delta_enc_left  * ENC_TICKS_TO_RADS / TIMESTEP;
  float wheel_vel_right = (float)delta_enc_right * ENC_TICKS_TO_RADS / TIMESTEP;

  // Check if tipped
  if (fabsf(pitch) > TIP_THRESHOLD) {
    tipped = true;
  }

  // Try to balance if not tipped
  if (!tipped) {
    // Build observation vector (must match training order):
    // [pitch, pitch_rate, wheel_vel_left, wheel_vel_right]
    float obs[ACTOR_OBS_SIZE] = {
      pitch + PITCH_OFFSET,
      pitch_rate,
      wheel_vel_left,
      wheel_vel_right
    };

    // Run inference (actor network forward pass)
    float action[ACTOR_ACTION_SIZE];
    uint32_t t4 = micros();
    actor_forward(obs, action);
    uint32_t t5 = micros();

    // Clamp actions to [-1, 1]
    action[0] = constrain(action[0], -1.0f, 1.0f);
    action[1] = constrain(action[1], -1.0f, 1.0f);

    // Use a low-pass filter to smooth out the motor commands
    action_filtered[0] = (ACTION_ALPHA * action_filtered[0]) +
                         ((1.0f - ACTION_ALPHA) * action[0]);
    action_filtered[1] = (ACTION_ALPHA * action_filtered[1]) +
                         ((1.0f - ACTION_ALPHA) * action[1]);

    // Apply action deadband to prevent small movements
    float action_motors[2];
    action_motors[0] = (fabs(action_filtered[0]) < ACTION_DEADBAND) ? 0.0f : action_filtered[0];
    action_motors[1] = (fabs(action_filtered[1]) < ACTION_DEADBAND) ? 0.0f : action_filtered[1];

    // Calculate actual motor values from normalized values (boost if needed)
    int16_t motor_left = MOTOR_DIR_LEFT *
                          (int16_t)(action_motors[0] * MOTOR_SCALE * MOTOR_BOOST);
    int16_t motor_right = MOTOR_DIR_RIGHT *
                          (int16_t)(action_motors[1] * MOTOR_SCALE * MOTOR_BOOST);

    // Set motor speed based on inference results
    bala.SetSpeed(motor_left, motor_right);
    uint32_t t6 = micros();

    // Record timing for this balancing step
    uint32_t work = t6 - (uint32_t)step_start;
    t_imu.add(t1 - t0);
    t_enc.add(t3 - t2);
    t_nn.add(t5 - t4);
    t_motor.add(t6 - t5);
    t_work.add(work);
    total_steps++;
    if (work > TIMESTEP_US) {
      n_overrun++;
      total_overrun++;
    }

  // If tipped, shut off motors and wait to be turned upright
  } else {
    bala.SetSpeed(0, 0);
    // Wait for someone to pick the bot up
    if (fabsf(pitch) <= 0.3) {
      tipped = false;

      // Reset the tip sensor pitch and filters
      pitch = 0.0f;
      action_filtered[0] = 0.0f;
      action_filtered[1] = 0.0f;

      // Reset the encoder counters
      bala.ClearEncoder();
      prev_enc_left = 0;
      prev_enc_right = 0;

      // Wait a moment before starting
      Serial.printf("Untipped! Starting in %lu seconds\n", RESET_TIME_MS / 1000);
      delay(RESET_TIME_MS);
      Serial.println("Running...");

      // Don't count the pause as a loop period, and start a fresh timing interval
      have_prev_step = false;
      reset_timing_stats();
      print_counter = 0;
    }
  }

  // Pace to TIMESTEP before printing
  while (micros() - step_start < TIMESTEP_US);

  // Print timing summary once per second while balancing. The print happens after the pacing
  // wait, and the next period is excluded from the stats so printing doesn't skew them.
  if (!tipped && ++print_counter >= TIMING_PRINT_STEPS) {
    print_counter = 0;
    print_timing_stats();
    reset_timing_stats();
    have_prev_step = false;
  }
}

#else  // TEST_MODE 1 or 2

void loop() {
  // Run the selected test, then wait for button A to run it again
#if TEST_MODE == 1
  run_step_tests();
#elif TEST_MODE == 2
  run_deadband_tests();
#endif
  set_motors(0.0f, 0.0f);
}

#endif
