/*
 * ARGUS — ESP32 Vehicle Firmware
 * 
 * WiFi UDP listener that receives JSON motor commands from the
 * Raspberry Pi ground station and sends IMU telemetry back.
 * 
 * Hardware:
 *   - ESP32 dev board
 *   - L298N motor driver (ENA, IN1, IN2, ENB, IN3, IN4)
 *   - MPU6050 IMU (I2C: SDA=21, SCL=22)
 *   - 2x DC gear motors (differential drive)
 * 
 * Communication:
 *   - Single UDP port 4210 (bidirectional)
 *   - Receives: {"type":"motor","left":0.0-1.0,"right":0.0-1.0,"dir":"fwd"/"rev","ts":ms}
 *   - Sends:    {"type":"imu","yaw":deg,"pitch":deg,"roll":deg,"ax":g,"ay":g,"az":g,...}
 *   - Failsafe: motors stop if no command received within 500ms
 * 
 * Board: ESP32 Dev Module
 * Upload speed: 921600
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <MPU6050_light.h>

// ═══════════════════════════════════════════════
// CONFIGURATION — EDIT THESE
// ═══════════════════════════════════════════════

// WiFi credentials
const char* WIFI_SSID     = "YOUR_WIFI_SSID";      // ← CHANGE THIS
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";   // ← CHANGE THIS

// Static IP configuration
IPAddress LOCAL_IP(192, 168, 1, 100);    // ← ESP32 static IP (match config.py)
IPAddress GATEWAY(192, 168, 1, 1);
IPAddress SUBNET(255, 255, 255, 0);

// Ground station (Pi 5) IP — for sending IMU data back
IPAddress GCS_IP(192, 168, 1, 50);       // ← CHANGE to your Pi's IP

// UDP port (bidirectional)
const uint16_t UDP_PORT = 4210;

// Motor driver pins (L298N)
const int ENA = 14;   // Left motor PWM (Enable A)
const int IN1 = 27;   // Left motor direction
const int IN2 = 26;   // Left motor direction
const int ENB = 25;   // Right motor PWM (Enable B)
const int IN3 = 33;   // Right motor direction
const int IN4 = 32;   // Right motor direction

// PWM configuration
const int PWM_FREQ     = 1000;   // 1 kHz
const int PWM_RES      = 8;      // 8-bit (0-255)
const int PWM_CH_LEFT  = 0;      // LEDC channel for left motor
const int PWM_CH_RIGHT = 1;      // LEDC channel for right motor

// IMU
const int IMU_SDA = 21;
const int IMU_SCL = 22;

// Timing
const unsigned long FAILSAFE_TIMEOUT_MS = 500;
const unsigned long IMU_SEND_INTERVAL_MS = 20;  // 50 Hz
const unsigned long HEARTBEAT_LED_MS = 100;

// Status LED
const int LED_PIN = 2;  // onboard LED

// ═══════════════════════════════════════════════
// GLOBALS
// ═══════════════════════════════════════════════

WiFiUDP udp;
MPU6050 mpu(Wire);

unsigned long lastCommandTime = 0;
unsigned long lastIMUSendTime = 0;
unsigned long lastHeartbeatTime = 0;
bool motorsActive = false;
bool imuReady = false;

// ═══════════════════════════════════════════════
// MOTOR CONTROL
// ═══════════════════════════════════════════════

void setupMotors() {
    // Direction pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    
    // PWM setup using LEDC
    ledcSetup(PWM_CH_LEFT, PWM_FREQ, PWM_RES);
    ledcSetup(PWM_CH_RIGHT, PWM_FREQ, PWM_RES);
    ledcAttachPin(ENA, PWM_CH_LEFT);
    ledcAttachPin(ENB, PWM_CH_RIGHT);
    
    stopMotors();
    Serial.println("[MOTORS] Initialized");
}

void setMotors(float leftPower, float rightPower, const char* dir) {
    // Convert 0.0-1.0 to 0-255 PWM
    int leftPWM  = constrain((int)(leftPower * 255), 0, 255);
    int rightPWM = constrain((int)(rightPower * 255), 0, 255);
    
    bool forward = (strcmp(dir, "fwd") == 0);
    
    // Left motor direction
    if (forward) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    }
    
    // Right motor direction
    if (forward) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    }
    
    // Set speed
    ledcWrite(PWM_CH_LEFT, leftPWM);
    ledcWrite(PWM_CH_RIGHT, rightPWM);
    
    motorsActive = (leftPWM > 0 || rightPWM > 0);
}

void stopMotors() {
    ledcWrite(PWM_CH_LEFT, 0);
    ledcWrite(PWM_CH_RIGHT, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    motorsActive = false;
}

// ═══════════════════════════════════════════════
// IMU
// ═══════════════════════════════════════════════

void setupIMU() {
    Wire.begin(IMU_SDA, IMU_SCL);
    Wire.setClock(400000);  // 400kHz I2C
    
    byte status = mpu.begin();
    if (status != 0) {
        Serial.printf("[IMU] ERROR: MPU6050 init failed (status %d)\n", status);
        imuReady = false;
        return;
    }
    
    Serial.println("[IMU] Calibrating gyroscope — keep vehicle still...");
    delay(1000);
    mpu.calcOffsets(true, true);  // gyro + accel offsets
    Serial.println("[IMU] Calibration complete");
    imuReady = true;
}

void sendIMUData() {
    if (!imuReady) return;
    
    mpu.update();
    
    StaticJsonDocument<256> doc;
    doc["type"]  = "imu";
    doc["yaw"]   = mpu.getAngleZ();
    doc["pitch"] = mpu.getAngleY();
    doc["roll"]  = mpu.getAngleX();
    doc["ax"]    = mpu.getAccX();
    doc["ay"]    = mpu.getAccY();
    doc["az"]    = mpu.getAccZ();
    doc["gx"]    = mpu.getGyroX();
    doc["gy"]    = mpu.getGyroY();
    doc["gz"]    = mpu.getGyroZ();
    doc["temp"]  = mpu.getTemp();
    doc["ts"]    = millis();
    
    char buffer[256];
    serializeJson(doc, buffer);
    
    udp.beginPacket(GCS_IP, UDP_PORT);
    udp.write((uint8_t*)buffer, strlen(buffer));
    udp.endPacket();
}

// ═══════════════════════════════════════════════
// NETWORK
// ═══════════════════════════════════════════════

void setupWiFi() {
    Serial.printf("[WIFI] Connecting to %s", WIFI_SSID);
    
    WiFi.mode(WIFI_STA);
    WiFi.config(LOCAL_IP, GATEWAY, SUBNET);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WIFI] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n[WIFI] FAILED — restarting...");
        ESP.restart();
    }
    
    udp.begin(UDP_PORT);
    Serial.printf("[UDP] Listening on port %d\n", UDP_PORT);
}

void sendAck(const char* ackType) {
    StaticJsonDocument<64> doc;
    doc["type"] = ackType;
    doc["ts"]   = millis();
    
    char buffer[64];
    serializeJson(doc, buffer);
    
    udp.beginPacket(GCS_IP, UDP_PORT);
    udp.write((uint8_t*)buffer, strlen(buffer));
    udp.endPacket();
}

void handleUDP() {
    int packetSize = udp.parsePacket();
    if (packetSize == 0) return;
    
    char buffer[512];
    int len = udp.read(buffer, sizeof(buffer) - 1);
    if (len <= 0) return;
    buffer[len] = '\0';
    
    // Update GCS IP from actual sender (in case it changed)
    GCS_IP = udp.remoteIP();
    
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, buffer);
    if (err) {
        Serial.printf("[UDP] JSON parse error: %s\n", err.c_str());
        return;
    }
    
    const char* msgType = doc["type"] | "";
    
    if (strcmp(msgType, "motor") == 0) {
        float left  = doc["left"]  | 0.0f;
        float right = doc["right"] | 0.0f;
        const char* dir = doc["dir"] | "fwd";
        
        setMotors(left, right, dir);
        lastCommandTime = millis();
        
        sendAck("ack");
        
    } else if (strcmp(msgType, "heartbeat") == 0) {
        sendAck("heartbeat_ack");
        lastHeartbeatTime = millis();
    }
}

// ═══════════════════════════════════════════════
// FAILSAFE
// ═══════════════════════════════════════════════

void checkFailsafe() {
    if (motorsActive && (millis() - lastCommandTime > FAILSAFE_TIMEOUT_MS)) {
        Serial.println("[FAILSAFE] No command received — stopping motors");
        stopMotors();
    }
}

// ═══════════════════════════════════════════════
// SETUP & LOOP
// ═══════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(500);
    
    Serial.println("\n=============================");
    Serial.println("  ARGUS Vehicle Controller");
    Serial.println("=============================\n");
    
    pinMode(LED_PIN, OUTPUT);
    
    setupMotors();
    setupIMU();
    setupWiFi();
    
    lastCommandTime = millis();
    
    Serial.println("\n[READY] Waiting for ground station commands...\n");
}

void loop() {
    // Handle incoming UDP commands
    handleUDP();
    
    // Check failsafe
    checkFailsafe();
    
    // Send IMU telemetry at 50Hz
    if (millis() - lastIMUSendTime >= IMU_SEND_INTERVAL_MS) {
        sendIMUData();
        lastIMUSendTime = millis();
    }
    
    // Status LED: solid when connected, blink when waiting
    if (millis() - lastCommandTime < 2000) {
        digitalWrite(LED_PIN, HIGH);  // solid = active
    } else {
        // Slow blink = waiting for connection
        digitalWrite(LED_PIN, (millis() / 500) % 2);
    }
    
    // WiFi reconnect
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WIFI] Disconnected — reconnecting...");
        stopMotors();
        WiFi.reconnect();
        delay(1000);
    }
}
