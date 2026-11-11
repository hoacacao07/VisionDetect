// ESP32 MOTOR + SENSOR CONTROLLER for the VisionDetection delivery robot.
//
// The Raspberry Pi asks for a speed per side; this board makes it happen and
// stays safe on its own:
//   * OPEN-LOOP motor speed (no wheel encoders): requested speed -> PWM duty
//     through the calibrated model in robot_config.h
//   * gyro heading hold so equal left/right requests drive straight
//   * 3 x HC-SR04 ultrasonic (trigger/echo), fired one at a time on core 0
//   * MPU-6050 gyro Z: bias calibrated at boot, integrated heading
//   * WATCHDOG: no valid command for COMMAND_TIMEOUT_MS -> brake, zero targets
//   * any malformed line -> brake immediately until the next valid command
//
// Protocol (text lines, 115200 baud):
//   Pi -> ESP32   C,<seq>,<left_mps>,<right_mps>
//   ESP32 -> Pi   T,<ms>,<estL_m>,<estR_m>,<rangeL_m>,<rangeC_m>,<rangeR_m>,<gyroZ_dps>,<heading_deg>
//                 estL/estR: ESTIMATED cumulative travel per side in metres,
//                 integrated from the speed this board applied (not measured).
//                 Missing/out-of-range/stale values are "nan", never 0.
//                 Lines starting with '#' are diagnostics.
//
// Non-blocking main loop on core 1; only the ultrasonic task (pulseIn) blocks,
// and it runs on core 0 so it cannot delay the speed loop or the watchdog.

#include <Arduino.h>
#include <Wire.h>
#include <esp_arduino_version.h>
#include <math.h>
#include "pins.h"
#include "robot_config.h"

constexpr uint32_t PWM_MAX = (1u << PWM_RESOLUTION_BITS) - 1;

// ============================== PWM =========================================
// PlatformIO's espressif32 platform ships Arduino-ESP32 2.x (channel API);
// the Arduino IDE may use 3.x (pin API). Support both.
static void pwmAttach(uint8_t pin, uint8_t channel) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcAttach(pin, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
#else
  ledcSetup(channel, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(pin, channel);
#endif
}

static void pwmWrite(uint8_t pin, uint8_t channel, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(channel, duty);
#endif
}

// ============================== MOTORS ======================================
struct Side {
  uint8_t enPin, in1Pin, in2Pin, channel;
  bool invert;
  float target;    // Requested by the Pi (m/s, signed).
  float setpoint;  // Ramped toward target.
  float applied;   // Speed the open-loop model expects right now (0 when braking).
  float travel;    // ESTIMATED cumulative travel, metres (integral of applied).

  // Explicit constructor: Arduino-ESP32 2.x compiles as C++11, where a struct
  // with member initializers cannot be brace-initialized as an aggregate.
  Side(uint8_t en, uint8_t in1, uint8_t in2, uint8_t ch, bool inv)
      : enPin(en), in1Pin(in1), in2Pin(in2), channel(ch), invert(inv), target(0),
        setpoint(0), applied(0), travel(0) {}
};

Side leftSide(PIN_LEFT_EN, PIN_LEFT_IN1, PIN_LEFT_IN2, PWM_CHANNEL_LEFT, LEFT_MOTOR_INVERT);
Side rightSide(PIN_RIGHT_EN, PIN_RIGHT_IN1, PIN_RIGHT_IN2, PWM_CHANNEL_RIGHT, RIGHT_MOTOR_INVERT);

void brake(Side &s) {
  // L298N fast stop: both inputs at the same level with EN high.
  digitalWrite(s.in1Pin, LOW);
  digitalWrite(s.in2Pin, LOW);
  pwmWrite(s.enPin, s.channel, PWM_MAX);
  s.applied = 0;
}

// Open-loop: drive at `speed` m/s (signed) and return the speed the model
// expects, which is capped at what full duty can deliver.
float drive(Side &s, float speed) {
  if (fabsf(speed) < 1e-3f) {
    brake(s);
    return 0;
  }
  int8_t dir = speed > 0 ? 1 : -1;
  bool forwardPins = (dir > 0) != s.invert;
  digitalWrite(s.in1Pin, forwardPins ? HIGH : LOW);
  digitalWrite(s.in2Pin, forwardPins ? LOW : HIGH);
  float effort = fminf(fabsf(speed) / SPEED_AT_FULL_DUTY_MPS, 1.0f);
  float duty = MIN_DUTY + (1.0f - MIN_DUTY) * effort;
  pwmWrite(s.enPin, s.channel, (uint32_t)(duty * PWM_MAX));
  return dir * effort * SPEED_AT_FULL_DUTY_MPS;
}

void stopNow(Side &s) {
  s.target = 0;
  s.setpoint = 0;
  brake(s);
}

// correction: heading-hold adjustment in m/s added to this side's setpoint.
void updateSide(Side &s, float dt, float correction) {
  // Ramp the setpoint within acceleration/deceleration limits.
  bool speedingUp = fabsf(s.target) > fabsf(s.setpoint) && s.target * s.setpoint >= 0;
  float step = (speedingUp ? MAX_ACCEL_MPS2 : MAX_DECEL_MPS2) * dt;
  if (s.target > s.setpoint) s.setpoint = fminf(s.target, s.setpoint + step);
  else s.setpoint = fmaxf(s.target, s.setpoint - step);

  float speed = s.setpoint + correction;
  // Stopping, or a correction that would reverse this side: hold with the brake.
  if (fabsf(s.setpoint) < 0.005f || speed * s.setpoint <= 0) {
    brake(s);
  } else {
    s.applied = drive(s, speed);
  }
  s.travel += s.applied * dt;
}

// Hold the gyro heading captured when a straight move starts. Returns the
// correction for the LEFT side (the right side gets the negative). Heading
// grows when turning left, so drifting left speeds up the left side.
bool headingHeld = false;
float headingTargetDeg = 0;

float headingCorrection(float headingDeg, bool imuOk) {
  bool straightForward = leftSide.target > 0 && leftSide.target == rightSide.target;
  if (!straightForward || !imuOk || isnan(headingDeg) || HEADING_HOLD_MPS_PER_DEG <= 0) {
    headingHeld = false;
    return 0;
  }
  if (!headingHeld) {
    headingHeld = true;
    headingTargetDeg = headingDeg;
  }
  float error = headingDeg - headingTargetDeg;
  while (error > 180.0f) error -= 360.0f;
  while (error <= -180.0f) error += 360.0f;
  return constrain(HEADING_HOLD_MPS_PER_DEG * error, -HEADING_HOLD_MAX_MPS, HEADING_HOLD_MAX_MPS);
}

// ============================ ULTRASONIC ====================================
portMUX_TYPE rangeMux = portMUX_INITIALIZER_UNLOCKED;
float rangeMetres[3] = {NAN, NAN, NAN};
uint32_t rangeAtMs[3] = {0, 0, 0};

void ultrasonicTask(void *) {
  TickType_t wake = xTaskGetTickCount();
  int sensor = 0;
  for (;;) {
    uint8_t trig = PIN_US_TRIG[sensor], echo = PIN_US_ECHO[sensor];
    digitalWrite(trig, LOW);
    delayMicroseconds(4);
    digitalWrite(trig, HIGH);
    delayMicroseconds(ULTRASONIC_TRIGGER_US);
    digitalWrite(trig, LOW);
    unsigned long echoUs = pulseIn(echo, HIGH, ULTRASONIC_TIMEOUT_US);
    float metres = NAN;  // No echo / timeout: unknown, never 0.
    if (echoUs > 0) {
      metres = echoUs * 1e-6f * SPEED_OF_SOUND_MPS / 2.0f;
      if (metres < ULTRASONIC_MIN_M || metres > ULTRASONIC_MAX_M) metres = NAN;
    }
    portENTER_CRITICAL(&rangeMux);
    rangeMetres[sensor] = metres;
    rangeAtMs[sensor] = millis();
    portEXIT_CRITICAL(&rangeMux);
    sensor = (sensor + 1) % 3;
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(ULTRASONIC_PERIOD_MS));  // One at a time.
  }
}

float readRange(int i, uint32_t nowMs) {
  portENTER_CRITICAL(&rangeMux);
  float metres = rangeMetres[i];
  uint32_t at = rangeAtMs[i];
  portEXIT_CRITICAL(&rangeMux);
  if (at == 0 || nowMs - at > RANGE_REPORT_MAX_AGE_MS) return NAN;
  return metres;
}

// ============================== IMU =========================================
bool imuOk = false;
float gyroBiasDps = 0;
float gyroZDps = NAN;
float headingDeg = 0;
int imuFailures = 0;

bool mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool mpuReadGyroZRaw(int16_t &raw) {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x47);  // GYRO_ZOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDRESS, 2) != 2) return false;
  raw = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

constexpr float GYRO_LSB_PER_DPS = 131.0f;  // +/-250 dps range.

bool imuBegin() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  if (!mpuWrite(0x6B, 0x01)) return false;  // Wake, PLL on gyro X.
  delay(50);
  if (!mpuWrite(0x1A, 0x03)) return false;  // DLPF ~44 Hz.
  if (!mpuWrite(0x1B, 0x00)) return false;  // +/-250 dps.
  delay(100);
  for (int attempt = 1; attempt <= IMU_CALIBRATION_ATTEMPTS; attempt++) {
    double sum = 0, sumSquares = 0;
    int good = 0;
    for (int i = 0; i < IMU_CALIBRATION_SAMPLES; i++) {
      int16_t raw;
      if (mpuReadGyroZRaw(raw)) {
        float dps = raw / GYRO_LSB_PER_DPS;
        sum += dps;
        sumSquares += dps * dps;
        good++;
      }
      delay(2);
    }
    if (good < IMU_CALIBRATION_SAMPLES * 9 / 10) return false;
    float mean = sum / good;
    float stdDev = sqrtf(fmaxf(0, sumSquares / good - mean * mean));
    if (stdDev <= IMU_STILL_MAX_STD_DPS) {
      gyroBiasDps = mean;
      Serial.printf("# imu calibrated bias=%.3f dps std=%.3f\n", mean, stdDev);
      return true;
    }
    Serial.printf("# imu moving during calibration (std=%.3f), retry %d\n", stdDev, attempt);
  }
  return false;
}

void updateImu(float dt) {
  if (!imuOk) return;
  int16_t raw;
  if (!mpuReadGyroZRaw(raw)) {
    if (++imuFailures >= IMU_MAX_CONSECUTIVE_FAILURES) {
      gyroZDps = NAN;  // The Pi treats a missing gyro as STOP.
      headingDeg = NAN;
    }
    return;
  }
  imuFailures = 0;
  float dps = raw / GYRO_LSB_PER_DPS - gyroBiasDps;
  if (IMU_Z_INVERT) dps = -dps;
  gyroZDps = dps;
  if (isnan(headingDeg)) return;  // A gap makes the heading unrecoverable until reboot.
  headingDeg += dps * dt;
  while (headingDeg > 180.0f) headingDeg -= 360.0f;
  while (headingDeg <= -180.0f) headingDeg += 360.0f;
}

// ============================ PROTOCOL ======================================
uint32_t lastValidCommandMs = 0;
bool commandValid = false;  // False at boot, after a timeout and after bad input.
char lineBuffer[MAX_LINE_LENGTH + 1];
size_t lineLength = 0;
bool lineOverflow = false;

void rejectInput() {
  // Malformed input: stop now; the next valid command resumes control.
  commandValid = false;
  stopNow(leftSide);
  stopNow(rightSide);
}

bool parseFloat(const char *text, float &value) {
  char *end;
  value = strtof(text, &end);
  return end != text && *end == '\0' && isfinite(value);
}

void handleLine(char *line) {
  // Exactly: C,<seq>,<left_mps>,<right_mps>
  char *fields[5];
  int count = 0;
  for (char *token = strtok(line, ","); token && count < 5; token = strtok(nullptr, ",")) {
    fields[count++] = token;
  }
  if (count != 4 || strcmp(fields[0], "C") != 0) return rejectInput();
  char *end;
  long seq = strtol(fields[1], &end, 10);
  if (end == fields[1] || *end != '\0' || seq < 0) return rejectInput();
  float left, right;
  if (!parseFloat(fields[2], left) || !parseFloat(fields[3], right)) return rejectInput();
  if (fabsf(left) > COMMAND_SANITY_MPS || fabsf(right) > COMMAND_SANITY_MPS) return rejectInput();
  leftSide.target = constrain(left, -MAX_SPEED_MPS, MAX_SPEED_MPS);
  rightSide.target = constrain(right, -MAX_SPEED_MPS, MAX_SPEED_MPS);
  lastValidCommandMs = millis();
  commandValid = true;
}

void readSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (lineOverflow) rejectInput();
      else if (lineLength > 0) {
        lineBuffer[lineLength] = '\0';
        handleLine(lineBuffer);
      }
      lineLength = 0;
      lineOverflow = false;
    } else if (lineLength < MAX_LINE_LENGTH) {
      lineBuffer[lineLength++] = c;
    } else {
      lineOverflow = true;
    }
  }
}

void appendValue(char *buffer, size_t size, float value, int decimals) {
  size_t used = strlen(buffer);
  if (isnan(value) || isinf(value)) snprintf(buffer + used, size - used, ",nan");
  else snprintf(buffer + used, size - used, ",%.*f", decimals, value);
}

void sendTelemetry(uint32_t nowMs) {
  char line[160];
  snprintf(line, sizeof(line), "T,%lu", (unsigned long)nowMs);
  appendValue(line, sizeof(line), leftSide.travel, 4);  // Estimated, metres.
  appendValue(line, sizeof(line), rightSide.travel, 4);
  for (int i = 0; i < 3; i++) appendValue(line, sizeof(line), readRange(i, nowMs), 3);
  appendValue(line, sizeof(line), imuOk ? gyroZDps : NAN, 2);
  appendValue(line, sizeof(line), imuOk ? headingDeg : NAN, 2);
  Serial.println(line);
}

// ============================== MAIN ========================================
void setupSide(Side &s) {
  pinMode(s.in1Pin, OUTPUT);
  pinMode(s.in2Pin, OUTPUT);
  pwmAttach(s.enPin, s.channel);
  brake(s);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  setupSide(leftSide);  // Motors braked before anything else.
  setupSide(rightSide);
  for (int i = 0; i < 3; i++) {
    pinMode(PIN_US_TRIG[i], OUTPUT);
    digitalWrite(PIN_US_TRIG[i], LOW);
    pinMode(PIN_US_ECHO[i], INPUT);
  }
  Serial.println("# esp32_robot booting; keep the robot still for gyro calibration");
  imuOk = imuBegin();
  if (!imuOk) Serial.println("# imu FAILED: gyro reported as nan, the Pi will not drive");
  xTaskCreatePinnedToCore(ultrasonicTask, "ultrasonic", 4096, nullptr, 1, nullptr, 0);
  Serial.println("# esp32_robot ready (open-loop speed, no encoders)");
}

void loop() {
  static uint32_t lastControlMs = millis(), lastImuMs = millis(), lastTelemetryMs = 0;
  readSerial();
  uint32_t nowMs = millis();

  if (commandValid && nowMs - lastValidCommandMs >= COMMAND_TIMEOUT_MS) {
    commandValid = false;  // Watchdog: the Pi went quiet.
    stopNow(leftSide);     // Brake now, not at the next control tick.
    stopNow(rightSide);
    Serial.println("# watchdog: no command, braking");
  }
  if (!commandValid) {
    leftSide.target = 0;
    rightSide.target = 0;
  }
  if (nowMs - lastImuMs >= IMU_PERIOD_MS) {
    updateImu((nowMs - lastImuMs) / 1000.0f);
    lastImuMs = nowMs;
  }
  if (nowMs - lastControlMs >= CONTROL_PERIOD_MS) {
    float dt = (nowMs - lastControlMs) / 1000.0f;
    lastControlMs = nowMs;
    if (!commandValid) {
      stopNow(leftSide);  // Watchdog / bad input: brake now, no ramp.
      stopNow(rightSide);
      headingCorrection(NAN, false);  // Forget the held heading.
    } else {
      float correction = headingCorrection(headingDeg, imuOk);
      updateSide(leftSide, dt, correction);
      updateSide(rightSide, dt, -correction);
    }
  }
  if (nowMs - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = nowMs;
    sendTelemetry(nowMs);
  }
}
