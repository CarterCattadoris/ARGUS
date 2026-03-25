
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

4. Ran the Flask server and connected from the laptop browser at `http://<pi-ip>:5000`.

### Output

> **[PLACEHOLDER: Screenshot of browser dashboard showing live webcam feed]**

> **[PLACEHOLDER: Terminal output showing Flask server running and client connected]**

---

## 2. ESP32-S3 UDP Firmware Bench Test

**Owner: Lucas Le**

Wrote and tested the ESP32-S3 firmware to listen for UDP motor command packets over WiFi and translate them to PWM signals for the L298N motor driver. Tested on the bench without the chassis assembled.

### Steps

1. Flashed firmware to ESP32-S3 using Arduino IDE. The device joins the local WiFi network and opens a UDP socket on port `4210`.
    
2. The firmware parses incoming JSON motor commands:
    

```cpp
// Expected packet format
// {"left": 0.75, "right": 0.75, "dir": "fwd"}

StaticJsonDocument<128> doc;
deserializeJson(doc, packet);
float left  = doc["left"];
float right = doc["right"];
String dir  = doc["dir"].as<String>();
```

3. Translates to PWM output on L298N enable pins:

```cpp
analogWrite(ENA, (int)(left  * 255));
analogWrite(ENB, (int)(right * 255));
digitalWrite(IN1, dir == "fwd" ? HIGH : LOW);
digitalWrite(IN2, dir == "fwd" ? LOW  : HIGH);
```

4. Sent test packets from the Pi using Python to verify correct behavior:

```python
import socket, json
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cmd = json.dumps({"left": 0.6, "right": 0.6, "dir": "fwd"})
sock.sendto(cmd.encode(), ("<esp32-ip>", 4210))
```

5. Verified the 500ms failsafe: if no command is received within 500ms, motors stop automatically.

### Output

> **[PLACEHOLDER: Serial monitor screenshot showing received UDP packets and PWM values]**


---

## 3. Jon-Micheal — Dashboard Controls Groundwork

**Owner: Jon-Micheal Gonzalez**

Began laying out the manual drive controls section of the dashboard UI (keyboard bindings and on-screen button layout) in preparation for Week 3 integration with the ESP32 firmware.

> **[PLACEHOLDER: Screenshot or code snippet of controls UI in progress]**

---

## Progress vs. Plan

|Planned (Week 2)|Status|
|---|---|
|CV pipeline: camera calibration, marker tracking, coordinate mapping|⏳ Deferred to Week 3|
|Flask dashboard with live video feed|✅ Complete|
|ESP32 firmware: UDP listener to motor PWM (bench tested)|✅ Complete|

Camera calibration and ArUco coordinate mapping were deprioritized this week to ensure the communication layer between the Pi and ESP32 was solid before chassis assembly begins in Week 3.

---

## Next Week (Mar 31 – Apr 6)

- Assemble 2WD chassis once parts arrive
- Wire motors, L298N, ESP32-S3, and MPU6050 IMU
- Deploy YOLO model on Pi and begin CV pipeline with coordinate mapping
- First end-to-end integration test: dashboard → Pi → ESP32 → motors
