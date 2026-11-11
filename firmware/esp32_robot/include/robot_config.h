// ROBOT CONSTANTS for the ESP32. Every hardware assumption lives here.
//
// NO WHEEL ENCODERS: motor speed is OPEN-LOOP. The ESP32 turns a requested
// speed into a PWM duty using the calibration below, and reports the travel it
// EXPECTS from that (an estimate, not a measurement) to the Pi.
// Calibrate SPEED_AT_FULL_DUTY_MPS on the floor with the real load (README,
// "Speed calibration"): battery charge, load and floor all change the result.
#pragma once
#include <stdint.h>

// The Python fake ESP32 (sim/fake_esp32.py) reads this file directly, so the
// simulation always uses these exact numbers. Keep each value a plain literal.
//
// ---- Open-loop speed model -------------------------------------------------
// Speed is modelled as 0 at MIN_DUTY and SPEED_AT_FULL_DUTY_MPS at 100 % duty,
// linear in between.
//
// Where the defaults come from (TT gear motor, 1:48, ~200 RPM no-load at 6 V,
// 65 mm wheel, 2S Li-ion motor battery 7.4 V nominal, L298N drop ~1.5-2 V):
//   no-load top speed at 6 V = 200 / 60 x pi x 0.065          = 0.68 m/s
//   motor voltage = 7.4 V - ~1.8 V (L298N)                     = ~5.6 V
//   no-load at 5.6 V ~ 0.68 x 5.6 / 6                           = ~0.63 m/s
//   under load (robot weight, gearbox friction) ~ 70-80 %      = ~0.45-0.50 m/s
// SPEED_AT_FULL_DUTY_MPS = 0.50 takes the HIGH end on purpose: if the real
// robot is slower than the model, it is slower than the Pi thinks, which is
// the safe direction for stopping distance.
// TT motors need roughly 2-2.5 V to start turning under load:
//   2.2 V / 5.6 V                                              = ~0.39
// MIN_DUTY = 0.40.
// BOTH VALUES MUST BE CALIBRATED on the real robot (README, "Speed calibration").
constexpr float MIN_DUTY = 0.40f;                // MUST CALIBRATE.
constexpr float SPEED_AT_FULL_DUTY_MPS = 0.50f;  // MUST CALIBRATE.
// Flip if a side drives backwards when commanded forward.
constexpr bool LEFT_MOTOR_INVERT = false;
constexpr bool RIGHT_MOTOR_INVERT = false;

// ---- Speed limits and ramp ---------------------------------------------------
constexpr float MAX_SPEED_MPS = 0.5f;        // Any command is clamped to this.
constexpr float COMMAND_SANITY_MPS = 3.0f;   // Larger values = malformed line.
constexpr float MAX_ACCEL_MPS2 = 0.6f;       // Firmware ramp; the Pi ramps gentler.
constexpr float MAX_DECEL_MPS2 = 1.2f;

// ---- Heading hold (gyro) -----------------------------------------------------
// Without encoders the two sides are not speed-matched, so the robot would
// curve. While both sides are asked for the same forward speed, the gyro
// heading at the start of the move is held by speeding one side up and slowing
// the other. Set HEADING_HOLD_MPS_PER_DEG to 0 to disable.
constexpr float HEADING_HOLD_MPS_PER_DEG = 0.01f;    // Correction per degree of drift.
constexpr float HEADING_HOLD_MAX_MPS = 0.05f;        // Largest per-side correction.

// ---- PWM (LEDC) -------------------------------------------------------------
constexpr uint32_t PWM_FREQUENCY_HZ = 1000;  // L298N switches slowly; keep <= ~1-2 kHz.
constexpr uint8_t PWM_RESOLUTION_BITS = 10;
constexpr uint8_t PWM_CHANNEL_LEFT = 0;      // Used on Arduino-ESP32 2.x only.
constexpr uint8_t PWM_CHANNEL_RIGHT = 1;

// ---- Ultrasonic (3 x HC-SR04, trigger/echo) ---------------------------------
// 10 us trigger pulse; echo pulse width = round-trip time. Fired one at a time
// so sensors never hear each other's echoes.
constexpr uint32_t ULTRASONIC_TRIGGER_US = 10;
constexpr uint32_t ULTRASONIC_PERIOD_MS = 30;     // One sensor per slot; 90 ms per full cycle.
constexpr uint32_t ULTRASONIC_TIMEOUT_US = 25000;  // ~4.3 m round trip; no echo -> nan.
constexpr float SPEED_OF_SOUND_MPS = 343.0f;       // ~20 C air.
constexpr float ULTRASONIC_MIN_M = 0.02f;          // Below/above: reported nan.
constexpr float ULTRASONIC_MAX_M = 4.0f;
constexpr uint32_t RANGE_REPORT_MAX_AGE_MS = 150;  // Older reading -> nan.

// ---- MPU-6050 -------------------------------------------------------------
constexpr uint8_t MPU_ADDRESS = 0x68;              // 0x69 if AD0 is tied high.
constexpr uint32_t IMU_PERIOD_MS = 10;
constexpr int IMU_CALIBRATION_SAMPLES = 400;       // ~1 s; robot must be still.
constexpr float IMU_STILL_MAX_STD_DPS = 0.5f;      // Retry if noisier (robot moved).
constexpr int IMU_CALIBRATION_ATTEMPTS = 5;
constexpr int IMU_MAX_CONSECUTIVE_FAILURES = 5;    // Then report nan (Pi stops).
// Heading must INCREASE when the robot turns LEFT (seen from above), or the
// heading hold steers the wrong way. Chip upside down -> true.
constexpr bool IMU_Z_INVERT = false;

// ---- Link / safety --------------------------------------------------------
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t COMMAND_TIMEOUT_MS = 300;       // No valid C line -> brake.
constexpr uint32_t CONTROL_PERIOD_MS = 20;         // 50 Hz motor update.
constexpr uint32_t TELEMETRY_PERIOD_MS = 50;       // 20 Hz to the Pi.
constexpr size_t MAX_LINE_LENGTH = 80;
