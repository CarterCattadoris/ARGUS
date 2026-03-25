"""
ARGUS — ESP32 Communications
Bidirectional UDP on a single port (default 4210):
  → Motor commands (JSON) to ESP32
  ← IMU telemetry (JSON) from ESP32
"""

import socket
import json
import time
import threading
import logging

log = logging.getLogger("argus.comms")


class ESP32Comms:
    def __init__(self, esp32_ip="192.168.1.100", udp_port=4210,
                 failsafe_timeout=500):
        self.esp32_ip = esp32_ip
        self.udp_port = udp_port
        self.failsafe_timeout_s = failsafe_timeout / 1000.0

        # UDP socket
        self._sock = None
        self._sock_lock = threading.Lock()

        # IMU state
        self._imu_data = {}
        self._imu_lock = threading.Lock()
        self._last_imu_time = 0.0
        self.imu_active = False

        # Connection tracking
        self._last_send_time = 0.0
        self._last_ack_time = 0.0
        self._connected = False

        # Thread control
        self._running = False
        self._listener_thread = None

    # ──────────────────────────────────────────
    # Startup / Shutdown
    # ──────────────────────────────────────────
    def start(self):
        """Create UDP socket and start listener thread."""
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind(("0.0.0.0", self.udp_port))
            self._sock.settimeout(0.1)  # 100ms receive timeout
            log.info(f"UDP socket bound to 0.0.0.0:{self.udp_port}")
        except Exception as e:
            log.error(f"Failed to create UDP socket: {e}")
            self._sock = None
            return

        self._running = True
        self._listener_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._listener_thread.start()

        # Send initial heartbeat
        self._send_heartbeat()

    def stop(self):
        """Stop listener and close socket."""
        self._running = False
        # Send stop command before closing
        self.send_motor_command(0.0, 0.0, "fwd")
        time.sleep(0.05)

        if self._listener_thread:
            self._listener_thread.join(timeout=2)
        if self._sock:
            self._sock.close()
            self._sock = None
        log.info("ESP32 comms stopped")

    # ──────────────────────────────────────────
    # Send Motor Commands
    # ──────────────────────────────────────────
    def send_motor_command(self, left, right, direction="fwd"):
        """
        Send motor command to ESP32.
        Args:
            left: 0.0 - 1.0 (left motor power)
            right: 0.0 - 1.0 (right motor power)
            direction: "fwd" or "rev"
        """
        if self._sock is None:
            return False

        cmd = {
            "type": "motor",
            "left": round(max(0.0, min(1.0, left)), 3),
            "right": round(max(0.0, min(1.0, right)), 3),
            "dir": direction,
            "ts": int(time.time() * 1000),  # millisecond timestamp
        }

        return self._send(cmd)

    def _send_heartbeat(self):
        """Send a heartbeat packet to detect ESP32 presence."""
        if self._sock is None:
            return
        cmd = {"type": "heartbeat", "ts": int(time.time() * 1000)}
        self._send(cmd)

    def _send(self, data):
        """Send JSON data to ESP32 via UDP."""
        try:
            payload = json.dumps(data).encode("utf-8")
            with self._sock_lock:
                self._sock.sendto(payload, (self.esp32_ip, self.udp_port))
            self._last_send_time = time.time()
            return True
        except Exception as e:
            log.warning(f"UDP send failed: {e}")
            return False

    # ──────────────────────────────────────────
    # Receive IMU Telemetry
    # ──────────────────────────────────────────
    def _listen_loop(self):
        """Listen for incoming UDP packets (IMU data, ACKs) from ESP32."""
        log.info("UDP listener started")
        heartbeat_interval = 1.0
        last_heartbeat = time.time()

        while self._running:
            try:
                data, addr = self._sock.recvfrom(1024)
                packet = json.loads(data.decode("utf-8"))

                msg_type = packet.get("type", "")

                if msg_type == "imu":
                    with self._imu_lock:
                        self._imu_data = {
                            "yaw": packet.get("yaw"),
                            "pitch": packet.get("pitch"),
                            "roll": packet.get("roll"),
                            "accel_x": packet.get("ax"),
                            "accel_y": packet.get("ay"),
                            "accel_z": packet.get("az"),
                            "gyro_x": packet.get("gx"),
                            "gyro_y": packet.get("gy"),
                            "gyro_z": packet.get("gz"),
                            "temp": packet.get("temp"),
                        }
                        self._last_imu_time = time.time()
                        self.imu_active = True

                elif msg_type == "ack":
                    self._last_ack_time = time.time()
                    self._connected = True

                elif msg_type == "heartbeat_ack":
                    self._last_ack_time = time.time()
                    self._connected = True
                    log.debug("ESP32 heartbeat ACK received")

            except socket.timeout:
                pass
            except json.JSONDecodeError:
                log.warning("Malformed UDP packet received")
            except Exception as e:
                if self._running:
                    log.error(f"UDP listener error: {e}")

            # Check IMU timeout
            from config import Config
            if time.time() - self._last_imu_time > Config.IMU_TIMEOUT_S:
                self.imu_active = False

            # Check connection timeout
            if time.time() - self._last_ack_time > 5.0:
                self._connected = False

            # Periodic heartbeat
            if time.time() - last_heartbeat > heartbeat_interval:
                self._send_heartbeat()
                last_heartbeat = time.time()

        log.info("UDP listener stopped")

    # ──────────────────────────────────────────
    # Public accessors
    # ──────────────────────────────────────────
    def get_imu_data(self):
        """Get latest IMU reading (thread-safe)."""
        with self._imu_lock:
            if not self._imu_data:
                return None
            return dict(self._imu_data)

    def is_connected(self):
        """Check if ESP32 has responded recently."""
        return self._connected
