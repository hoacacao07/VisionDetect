# Wiring: delivery robot

Signal chain: **Raspberry Pi 5 → USB serial → ESP32 → L298N → TT motors → wheels.**
Pin numbers come from `firmware/esp32_robot/include/pins.h`. If you change a pin, change it there and here. `tests/test_hardware_consistency.py` checks that every pin exists on the 30-pin board and avoids the flash, strapping and USB pins.

## Safety rules (read first)

1. **Put a voltage divider on every HC-SR04 Echo line.** The HC-SR04 runs on 5 V and its Echo pin outputs 5 V, but ESP32 GPIO pins only take 3.3 V. Wire it like this:

   ```text
   HC-SR04 Echo ──[ 1 kΩ ]──┬── ESP32 GPIO (34 / 35 / 36)
                            │
                         [ 2 kΩ ]
                            │
                           GND
   ```

   This gives 5 V × 2k / (1k + 2k) = **3.33 V**. Trig needs no divider, because the ESP32's 3.3 V is enough to trigger the HC-SR04.

2. **Use one common ground.** Connect the motor battery −, both L298N GND terminals, the ESP32 GND, all HC-SR04 GND pins and the MPU-6050 GND together. The Pi is connected to that ground through the USB cable. Without a common ground, the PWM and Echo signals have no reference and behave randomly.

3. **Power the motors from their own battery,** for example 2S Li-ion (7.4 V nominal, 8.4 V full) or 6 × AA. Never power the motors from the Pi or the ESP32. TT motors draw 0.2–1 A each, and a stall or startup spike would reset the Pi or the ESP32.

4. **Expect the L298N to drop about 1.5–2 V.** It uses bipolar transistors, so the motors get less than the battery voltage:

   | Battery | Battery voltage | Motor voltage at 100 % PWM |
   | --- | --- | --- |
   | 2S Li-ion, full | 8.4 V | ≈ 6.6 V (a bit above the TT motor's 6 V rating; the firmware caps speed at `MAX_SPEED_MPS`) |
   | 2S Li-ion, nominal | 7.4 V | ≈ 5.6 V (used for the `robot_config.h` defaults) |
   | 2S Li-ion, near empty | 6.4 V | ≈ 4.6 V (slower; recalibrate or recharge) |
   | 6 × AA alkaline / NiMH | 9 V / 7.2 V | ≈ 7.2 V / 5.4 V (alkaline is too high for long runs at full duty) |

   Because the motor voltage changes with battery charge and there are no encoders, recalibrate `SPEED_AT_FULL_DUTY_MPS` with a charged battery.

5. **Leave the L298N 5 V regulator jumper (5V-EN) on** for a 2S or 6 × AA battery, which is anything up to 12 V. The on-board 78M05 then makes the L298N's 5 V logic supply from VS. With the jumper on, the **+5V terminal is an output**: never connect another 5 V supply to it. The 78M05 needs about 7 V in, so below about 6.6 V on a 2S pack the logic voltage starts to sag. Recharge at that point. Remove the jumper (and feed 5 V into +5V) only if the motor battery is above 12 V.

6. **Power the MPU-6050 (GY-521) from the ESP32's 3.3 V pin.** I2C then runs at 3.3 V.

7. **Fit 10 kΩ pull-downs from GPIO 25 and GPIO 13 to GND,** so the motors stay off while the ESP32 boots.

8. **Add an e-stop:** a latching switch in the motor battery + line, after the fuse. It cuts only the motors, so the Pi and ESP32 keep running and logging.

## Power

| From | To | Notes |
| --- | --- | --- |
| Motor battery + → fuse (3 A) → e-stop/switch | L298N #1 **+12V/VS**, L298N #2 **+12V/VS** | 2S Li-ion or 6 × AA. Motors only. 5V-EN jumper **on** |
| Motor battery − | L298N #1 GND, L298N #2 GND, **common ground** | |
| Pi 5 power supply, 5 V / 5 A USB-C | Pi 5 USB-C | Separate from the motor battery: the official 27 W PSU on the bench, a 5 V/5 A USB-C PD power bank on the robot |
| Pi 5 USB-A port | ESP32 USB-C (CP2102) | Powers the ESP32 and carries serial data. Shows up as `/dev/ttyUSB0` |
| ESP32 **VIN** (5 V from USB) | 3 × HC-SR04 VCC | About 15 mA each. Check that VIN measures at least 4.5 V |
| ESP32 **3V3** | MPU-6050 VCC | |
| ESP32 GND | common ground | Also the HC-SR04 GND, MPU-6050 GND and divider GND |

## ESP32 (30-pin DevKit) pin table

| GPIO | Direction | Connects to | Notes |
| --- | --- | --- | --- |
| 25 | out (PWM) | L298N #1 **ENA and ENB** | Left speed. 10 kΩ to GND. ENA/ENB jumpers removed |
| 26 | out | L298N #1 **IN1 and IN3** | Left direction |
| 27 | out | L298N #1 **IN2 and IN4** | Left direction |
| 13 | out (PWM) | L298N #2 **ENA and ENB** | Right speed. 10 kΩ to GND. ENA/ENB jumpers removed |
| 32 | out | L298N #2 **IN1 and IN3** | Right direction |
| 33 | out | L298N #2 **IN2 and IN4** | Right direction |
| 16 | out | HC-SR04 **left** Trig | |
| 17 | out | HC-SR04 **centre** Trig | |
| 4 | out | HC-SR04 **right** Trig | |
| 34 | in only | HC-SR04 **left** Echo, **through the 1k/2k divider** | Input-only pin |
| 35 | in only | HC-SR04 **centre** Echo, **through the 1k/2k divider** | Input-only pin |
| 36 (VP) | in only | HC-SR04 **right** Echo, **through the 1k/2k divider** | Input-only pin |
| 21 | I2C SDA | MPU-6050 SDA | |
| 22 | I2C SCL | MPU-6050 SCL | |
| 1 / 3 | UART0 | CP2102 → USB → Pi | Used by USB serial. Don't connect anything |

Spare pins: 39 (VN, input only), 14, 18, 19, 23. Not used: 6–11 (flash) and 0, 2, 5, 12, 15 (strapping pins, which can stop the board booting).

## L298N boards and motors

| L298N | Terminal | Connects to |
| --- | --- | --- |
| #1 (left side) | OUT1 / OUT2 | Left **front** TT motor |
| #1 | OUT3 / OUT4 | Left **rear** TT motor |
| #1 | ENA + ENB (jumpers removed) | ESP32 GPIO 25 (both pins) |
| #1 | IN1 + IN3 | ESP32 GPIO 26 |
| #1 | IN2 + IN4 | ESP32 GPIO 27 |
| #2 (right side) | OUT1 / OUT2 | Right **front** TT motor |
| #2 | OUT3 / OUT4 | Right **rear** TT motor |
| #2 | ENA + ENB (jumpers removed) | ESP32 GPIO 13 (both pins) |
| #2 | IN1 + IN3 | ESP32 GPIO 32 |
| #2 | IN2 + IN4 | ESP32 GPIO 33 |

Both motors on one side get identical signals, so they must turn the same way. If one turns backwards, swap **that motor's** two wires. If a whole side runs backwards, set `LEFT_MOTOR_INVERT` or `RIGHT_MOTOR_INVERT` in `robot_config.h`.

## HC-SR04 sensors

| Sensor | VCC | Trig | Echo (through the divider) | GND | Mounting |
| --- | --- | --- | --- | --- | --- |
| Left | ESP32 VIN (5 V) | GPIO 16 | GPIO 34 | common | Front bumper, 5 cm left of centre, angled 15° outward |
| Centre | ESP32 VIN (5 V) | GPIO 17 | GPIO 35 | common | Front bumper, centre, straight ahead |
| Right | ESP32 VIN (5 V) | GPIO 4 | GPIO 36 (VP) | common | Front bumper, 5 cm right of centre, angled 15° outward |

The firmware fires one sensor every 30 ms (one full cycle every 90 ms), so the sensors never hear each other's echoes. No echo within 25 ms (about 4 m) becomes `nan`, and `nan` means STOP. The angled sensors' beams are about 15° wide, so in a corridor narrower than about 1.6 m they see the side walls closer than `slow_gap_m` and the robot drives slowly. The simulation shows the same effect.

## MPU-6050 (GY-521)

| GY-521 pin | Connects to |
| --- | --- |
| VCC | ESP32 3V3 |
| GND | common ground |
| SCL | GPIO 22 |
| SDA | GPIO 21 |
| AD0 | leave unconnected (address 0x68) |

Mount it flat, chip side up, near the middle of the robot. When the robot turns **left** (seen from above), the heading must **increase**. The heading hold depends on this, so check it in bring-up step 1.

## Camera

| Part | Connects to |
| --- | --- |
| Camera Module 3 Wide (IMX708, 102° horizontal) | Pi 5 **CAM/DISP 0** (or 1) with the **22-pin to 15-pin** FPC cable: the 22-pin narrow end goes to the Pi, the 15-pin wide end to the camera |

Power the Pi off before connecting the cable. On the Pi 5 connector the contacts face the USB/Ethernet ports. On the camera they face the sensor board. Mount the camera upright on the robot's centre line. Measure its height above the floor and its downward tilt, then enter them in `config/cameras/robot.json`.
