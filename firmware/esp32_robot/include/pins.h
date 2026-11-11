// PIN MAP: ESP32-WROOM-32 DevKit, 30-PIN version (USB-C, CP2102).
// The single source of wiring truth; WIRING.md has the same table.
//
// Every pin below is broken out on the 30-pin board.
// Avoided: 6-11 (SPI flash, not broken out on the 30-pin board),
//          0/2/5/12/15 (boot strapping), 1/3 (USB serial to the Pi),
//          14 (spare; outputs PWM during boot).
// 34/35/36/39 are INPUT-ONLY with no internal pull-ups: ideal for HC-SR04
// Echo, which drives the line itself (through the 1k/2k voltage divider).
#pragma once
#include <stdint.h>

// ---- L298N #1 = LEFT pair, L298N #2 = RIGHT pair -------------------------
// Each side's two motors share one set of signals: wire the ESP32 EN pin to
// BOTH ENA and ENB of that side's L298N, IN1 to IN1+IN3, IN2 to IN2+IN4.
// Add a 10 kOhm pull-down from each EN pin to GND so motors stay off while the
// ESP32 boots (pins float until setup() runs).
constexpr uint8_t PIN_LEFT_EN = 25;    // PWM (LEDC)
constexpr uint8_t PIN_LEFT_IN1 = 26;
constexpr uint8_t PIN_LEFT_IN2 = 27;
constexpr uint8_t PIN_RIGHT_EN = 13;   // PWM (LEDC)
constexpr uint8_t PIN_RIGHT_IN1 = 32;
constexpr uint8_t PIN_RIGHT_IN2 = 33;

// ---- 3 x HC-SR04 ultrasonic, 5 V supply ------------------------------------
// Index order is the protocol order: 0 = left, 1 = centre, 2 = right.
// Trig: 3.3 V from the ESP32 is enough for the HC-SR04 trigger input.
// Echo: the HC-SR04 outputs 5 V. ALWAYS use a divider (1 kOhm from Echo,
// 2 kOhm to GND, ESP32 pin at the junction -> 3.3 V). 5 V damages the ESP32.
constexpr uint8_t PIN_US_TRIG[3] = {16, 17, 4};
constexpr uint8_t PIN_US_ECHO[3] = {34, 35, 36};  // 36 is labelled VP on the board

// ---- MPU-6050 (GY-521) over I2C, VCC from 3.3 V --------------------------
constexpr uint8_t PIN_I2C_SDA = 21;
constexpr uint8_t PIN_I2C_SCL = 22;

// Spare: 39 (VN, input-only), 14, 18, 19, 23.
