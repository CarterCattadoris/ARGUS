
**Group Members:** Carter, Lucas Le, Jon-Micheal Gonzalez  
**Course:** CSE 398 Junior Design — Spring 2026  
**Week:** Mar 24–30

---

## Hardware Used

|Name|Role|
|---|---|
|Raspberry Pi 5|Ground station — ran Flask server and CV pipeline|
|USB Webcam|Live video feed input to Pi|
|ESP32-S3 Dev Board|Vehicle MCU — bench tested UDP listener and motor PWM output|
|Laptop|Dashboard browser client, SSH terminal into Pi|

---

## Summary of Work

This week focused on two major deliverables: getting a live Flask dashboard up and running with a streaming webcam feed, and bench-testing the ESP32-S3 firmware for UDP motor command reception. All three team members contributed across these two tracks.

---

## 1. Flask Dashboard with Live Video Feed

**Owner: Carter**

Set up the Flask + Flask-SocketIO web server on the Raspberry Pi 5 served to the laptop browser. The dashboard streams live webcam frames over a WebSocket connection and displays them in real time.

### Steps

1. Installed dependencies on Pi:

```bash
pip install flask flask-socketio opencv-python-headless --break-system-packages
```

2. Captured frames from the USB webcam using OpenCV:

```python
import cv2
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
```

3. Encoded frames as JPEG and emitted over SocketIO:

```python
_, buffer = cv2.imencode('.jpg', frame)
frame_bytes = buffer.tobytes()
socketio.emit('frame', frame_bytes)
```

4. Ran the Flask server and connected from the laptop browser at `http://127.0.0.1:5000`.

### Output

>![[web-dashboard-photo-3-25.png]]
>


---

## 2. ESP32-S3 Firmware Development & Bench Test

Owner: Carter

Set up the ESP-IDF v6.0 development environment on Linux Mint and wrote modular ESP32-S3 firmware in C. The firmware connects to WiFi, listens for UDP motor command packets, parses JSON payloads, and stores them in a thread-safe shared command struct using FreeRTOS mutexes.
Environment Setup

Installed ESP-IDF v6.0 via Espressif Installation Manager (EIM) on Linux Mint.
Configured the toolchain for the ESP32-S3 target and verified the build/flash/monitor workflow over USB-Serial/JTAG (/dev/ttyACM0).
Built and flashed a "Hello from ARGUS" test to confirm the full toolchain was operational.



### WiFi Station Mode

Implemented WiFi station mode with automatic connection and retry logic. The ESP32-S3 connects to the lab WiFi network and prints its assigned IP address on success.
Registered event handlers for WiFi start, disconnect (with retry up to 5 attempts), and IP assignment — modeled after ESP-IDF's station example.


![[Monitor_output_esp_1.png]]

### UDP Listener & JSON Command Parsing

Created a FreeRTOS task for UDP reception on port 9876. The task opens a socket, binds, and loops on recvfrom() to receive incoming packets.
Integrated the cJSON library to parse incoming motor command packets in the format:

json{"left": 0.5, "right": 0.7, "dir": 1}

Parsed values are stored in a shared motor_cmd_t struct protected by a FreeRTOS mutex, along with a microsecond timestamp for failsafe timeout tracking.
Tested by sending UDP packets from a laptop using a Python script:

pythonimport socket, json
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.sendto(json.dumps({"left": 0.5, "right": 0.7, "dir": 1}).encode(), ("10.144.113.116", 9876))

Terminal Output:
I (1659) wifi: Connected — IP: 10.144.113.116
I (6039) udp_listener: [10.144.113.51:36767] {"left": 0.5, "right": 0.7, "dir": 1}
I (6039) udp_listener: CMD: L=0.50 R=0.70 D=1


### Project Structure

Organized firmware into modular files for maintainability:

Embedded/
├── CMakeLists.txt
├── main/
│   ├── CMakeLists.txt
│   ├── main.c          // app_main, task creation
│   ├── wifi.c / wifi.h // WiFi station init, event handler
│   ├── udp.c / udp.h   // UDP listener task, JSON parsing
│   └── command.c / command.h  // Shared motor command state, mutex

![[project_structure.png]]

---

## Progress vs. Plan

| Planned (Week 2)                                                     | Status               |
| -------------------------------------------------------------------- | -------------------- |
| CV pipeline: camera calibration, marker tracking, coordinate mapping | ⏳ Deferred to Week 3 |
| Flask dashboard with live video feed                                 | ✅ Complete           |
| ESP32 firmware: UDP listener to motor PWM (bench tested)             | ✅ Complete           |
|                                                                      |                      |

Camera calibration and ArUco coordinate mapping were deprioritized this week to ensure the communication layer between the Pi and ESP32 was solid before chassis assembly begins in Week 3.

---

## Next Week (Mar 31 – Apr 6)

- Assemble 2WD chassis once parts arrive
- Wire motors, L298N, ESP32-S3, and MPU6050 IMU
- Deploy YOLO model on Pi and begin CV pipeline with coordinate mapping
- First end-to-end integration test: dashboard → Pi → ESP32 → motors
