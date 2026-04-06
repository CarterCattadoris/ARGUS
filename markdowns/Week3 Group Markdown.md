**Group Members:** Carter, Lucas Le, Jon-Micheal Gonzalez  
**Course:** CSE 398 Junior Design — Spring 2026  
**Week 3:** Mar 31 – Apr 6

---

## Hardware Used

| Name | Role |
|---|---|
| Raspberry Pi 5 | Ground station — CV pipeline, Flask dashboard |
| USB Webcam | Overhead video feed for YOLO detection |
| ESP32-S3 DevKitC-1 (WROOM-2) | Vehicle MCU — motor control via WiFi UDP |
| L298N Motor Driver | Dual H-bridge for 2WD differential drive |
| 2WD Robot Chassis | Vehicle platform with DC gear motors and caster wheel |
| 4x AA Battery Pack | Motor power supply (~6V) |
| Laptop | Dashboard client, ESP32 programming, UDP testing |

---

## Summary of Work

This week focused on assembling the vehicle hardware, writing and testing motor control firmware, training the YOLO detection model, and beginning end-to-end integration between the ground station and the car.

---

## 1. Vehicle Assembly & Motor Control Firmware

**Owner: Carter**

### Chassis Assembly & Wiring

Assembled the 2WD chassis kit and wired the L298N motor driver to the ESP32-S3. Resolved several hardware integration challenges:

- Wired L298N to ESP32-S3 with PWM on IN1/IN3 pins and plain GPIO on IN2/IN4 for direction control, with ENA/ENB jumpers tied HIGH.
- Initial approach of PWM on ENA/ENB pins failed due to 3.3V logic from the ESP32-S3 not reliably driving the L298N enable pins. Switched to IN-pin PWM control as the working architecture.
- Designed power architecture: AA battery pack feeds L298N motor supply directly; buck converter (set to 5V) powers ESP32 separately; shared ground between domains.

> **[SCREENSHOT: Photo of assembled car showing ESP32-S3, L298N, battery pack, and motor wiring]**

### Motor Driver Firmware (motor.c / motor.h)

Wrote the complete LEDC PWM motor driver module:

- Configured LEDC timer at 1kHz, 10-bit resolution (1024 duty levels) in low speed mode (only mode available on ESP32-S3).
- Two LEDC channels: one for left motor (IN1, GPIO 16), one for right motor (IN3, GPIO 8).
- Direction pins (IN2/IN4) configured as plain GPIO outputs.
- `motor_init()` — configures LEDC timer, channels, and GPIO direction pins.
- `motor_set(float left, float right)` — accepts signed floats (-1.0 to 1.0), handles per-motor direction independently, includes 0.05 deadband to prevent drift at zero.
- `motor_stop()` — kills all PWM and pulls direction pins LOW.
- `motor_task()` — FreeRTOS task that polls the shared command struct at 50Hz and applies motor commands.

![[ESPinit.png]]
![[MotorTerminalOutput.png]]


### Command Format Update

Updated the motor command protocol from `{"left": 0.5, "right": 0.7, "dir": 1}` to signed floats: `{"left": 0.5, "right": -0.3}` where positive = forward, negative = reverse, per motor independently. Updated `motor_cmd_t` struct, `command.c`, and `udp.c` JSON parsing accordingly.

### UDP Motor Command Testing

Tested end-to-end motor control over WiFi UDP from raspberry pi Python script. Verified forward, reverse, turning, and variable speed control.


### CMakeLists.txt Updates

Added `esp_driver_ledc` and `esp_driver_gpio` to `PRIV_REQUIRES` in `main/CMakeLists.txt` for ESP-IDF v6.0 component dependencies.

---

## 2. 
---

## Current Firmware Status

| Module | File | Status |
|---|---|---|
| WiFi Station Mode | wifi.c / wifi.h | ✅ Complete |
| UDP Listener | udp.c / udp.h | ✅ Complete |
| Command State (mutex) | command.c / command.h | ✅ Complete |
| Motor PWM Driver | motor.c / motor.h | ✅ Complete |
| Motor Task (FreeRTOS) | motor_task in motor.c | ✅ Complete |
| IMU (MPU6050 I2C) | imu.c / imu.h | TODO |
| Watchdog Failsafe | watchdog.c / watchdog.h | TODO |

---

## Next Week Plan (Apr 7–13)

- **Carter:** Implement IMU module (MPU6050 I2C reads, UDP streaming to Pi), implement watchdog failsafe task, resolve any remaining motor control issues.
- **Lucas:** Deploy YOLO model on Pi 5 via NCNN, integrate detection with Flask dashboard overlay, retrain model with fully assembled car images.
- **Jon-Micheal:** PID autopilot controller implementation, dashboard telemetry and manual drive controls.
- **Integration:** First full end-to-end test — manual drive from dashboard through Pi → UDP → ESP32 → motors, with YOLO tracking overlay on the live feed.