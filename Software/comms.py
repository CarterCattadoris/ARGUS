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
import subprocess
import re

log = logging.getLogger("argus.comms")


def find_esp32_by_mac(mac_address, timeout=10):
    """
    Scan the ARP table to find the IP of a device by MAC address.
    Pings the broadcast address first to populate the ARP cache.
    Returns the IP string or None if not found.
    """
    mac_lower = mac_address.lower().strip()
    log.info(f"Scanning for ESP32 MAC: {mac_lower}")

    # 1) Try to populate ARP cache with a subnet ping
    try:
        # Get the local subnet from ip route
        result = subprocess.run(
            ["ip", "route", "show", "default"],
            capture_output=True, text=True, timeout=5
        )
        # Also do a broadcast ping on common interfaces
        subprocess.run(
            ["ping", "-b", "-c", "3", "-W", "1", "255.255.255.255"],
            capture_output=True, text=True, timeout=5,
            stderr=subprocess.DEVNULL
        )
    except Exception as e:
        log.debug(f"Broadcast ping attempt: {e}")

    # 2) Also try arp-scan if available (more reliable)
    try:
        result = subprocess.run(
            ["arp-scan", "-l", "-q"],
            capture_output=True, text=True, timeout=timeout
        )
        for line in result.stdout.splitlines():
            if mac_lower in line.lower():
                parts = line.split()
                if parts:
                    ip = parts[0]
                    log.info(f"Found ESP32 via arp-scan: {ip}")
                    return ip
    except FileNotFoundError:
        log.debug("arp-scan not installed, using /proc/net/arp")
    except Exception as e:
        log.debug(f"arp-scan failed: {e}")

    # 3) Check the kernel ARP table directly
    try:
        with open("/proc/net/arp", "r") as f:
            for line in f.readlines()[1:]:  # skip header
                parts = line.split()
                if len(parts) >= 4:
                    ip_addr = parts[0]
                    hw_addr = parts[3].lower()
                    if hw_addr == mac_lower:
                        log.info(f"Found ESP32 via ARP table: {ip_addr}")
                        return ip_addr
    except Exception as e:
        log.debug(f"ARP table read failed: {e}")

    # 4) Fallback: parse `arp -a` output
    try:
        result = subprocess.run(
            ["arp", "-a"],
            capture_output=True, text=True, timeout=5
        )
        for line in result.stdout.lower().splitlines():
            if mac_lower in line:
                match = re.search(r'\((\d+\.\d+\.\d+\.\d+)\)', line)
                if match:
                    ip = match.group(1)
                    log.info(f"Found ESP32 via arp -a: {ip}")
                    return ip
    except Exception as e:
        log.debug(f"arp -a failed: {e}")

    log.warning(f"ESP32 MAC {mac_lower} not found on network")
    return None


class ESP32Comms:
    def __init__(self, esp32_ip="192.168.1.100", udp_port=4210,
                 imu_port=9877, failsafe_timeout=500):
        self.esp32_ip = esp32_ip
        self.udp_port = udp_port
        self.imu_port = imu_port
        self.failsafe_timeout_s = failsafe_timeout / 1000.0

        # UDP sockets
        self._sock = None       # motor commands (port 9876)
        self._imu_sock = None   # IMU telemetry  (port 9877)
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
        self._imu_listener_thread = None

    # ──────────────────────────────────────────
    # Startup / Shutdown
    # ──────────────────────────────────────────
    def start(self):
        """Create UDP sockets and start listener threads."""
        # Motor command socket (port 9876)
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind(("0.0.0.0", self.udp_port))
            self._sock.settimeout(0.1)
            log.info(f"Motor UDP socket bound to 0.0.0.0:{self.udp_port}")
        except Exception as e:
            log.error(f"Failed to create motor UDP socket: {e}")
            self._sock = None

        # IMU telemetry socket (port 9877)
        try:
            self._imu_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._imu_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._imu_sock.bind(("0.0.0.0", self.imu_port))
            self._imu_sock.settimeout(0.1)
            log.info(f"IMU UDP socket bound to 0.0.0.0:{self.imu_port}")
        except Exception as e:
            log.error(f"Failed to create IMU UDP socket: {e}")
            self._imu_sock = None

        self._running = True
        self._listener_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._listener_thread.start()
        self._imu_listener_thread = threading.Thread(target=self._imu_listen_loop, daemon=True)
        self._imu_listener_thread.start()

    def stop(self):
        """Stop listeners and close sockets."""
        self._running = False
        self.send_motor_command(0.0, 0.0)
        time.sleep(0.05)

        if self._listener_thread:
            self._listener_thread.join(timeout=2)
        if self._imu_listener_thread:
            self._imu_listener_thread.join(timeout=2)
        if self._sock:
            self._sock.close()
            self._sock = None
        if self._imu_sock:
            self._imu_sock.close()
            self._imu_sock = None
        log.info("ESP32 comms stopped")

    # ──────────────────────────────────────────
    # Send Motor Commands
    # ──────────────────────────────────────────
    def send_motor_command(self, left, right):
        """
        Send motor command to ESP32.
        Args:
            left:  -1.0 (full reverse) to 1.0 (full forward)
            right: -1.0 (full reverse) to 1.0 (full forward)
        Sign of each value encodes that motor's direction.
        """
        if self._sock is None:
            return False

        left  = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        cmd = {
            "left":  round(left, 3),
            "right": round(right, 3),
        }
        return self._send(cmd)

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
    # Receive on motor port (echoes / ACKs)
    # ──────────────────────────────────────────
    def _listen_loop(self):
        """Listen for echo/ACK packets on the motor port."""
        log.info("Motor UDP listener started")

        while self._running:
            try:
                data, addr = self._sock.recvfrom(1024)
                packet = json.loads(data.decode("utf-8"))

                # ESP32 echoes back motor commands as ACK
                if "left" in packet and "right" in packet:
                    self._last_ack_time = time.time()
                    self._connected = True

            except socket.timeout:
                pass
            except json.JSONDecodeError:
                pass
            except Exception as e:
                if self._running:
                    log.error(f"Motor listener error: {e}")

            if time.time() - self._last_ack_time > 5.0:
                self._connected = False

        log.info("Motor UDP listener stopped")

    # ──────────────────────────────────────────
    # Receive IMU Telemetry (port 9877)
    # ──────────────────────────────────────────
    def _imu_listen_loop(self):
        """Listen for IMU telemetry from ESP32 on dedicated port."""
        log.info("IMU UDP listener started")

        while self._running:
            try:
                data, addr = self._imu_sock.recvfrom(1024)
                packet = json.loads(data.decode("utf-8"))

                # ESP32 firmware sends: {ax, ay, az, gx, gy, gz}
                with self._imu_lock:
                    self._imu_data = {
                        "accel_x": packet.get("ax"),
                        "accel_y": packet.get("ay"),
                        "accel_z": packet.get("az"),
                        "gyro_x": packet.get("gx"),
                        "gyro_y": packet.get("gy"),
                        "gyro_z": packet.get("gz"),
                    }
                    self._last_imu_time = time.time()
                    self.imu_active = True

            except socket.timeout:
                pass
            except json.JSONDecodeError:
                log.warning("Malformed IMU packet received")
            except Exception as e:
                if self._running:
                    log.error(f"IMU listener error: {e}")

            from config import Config
            if time.time() - self._last_imu_time > Config.IMU_TIMEOUT_S:
                self.imu_active = False

        log.info("IMU UDP listener stopped")

    # ──────────────────────────────────────────
    # Public accessors
    # ──────────────────────────────────────────
    def get_imu_data(self):
        """Get latest IMU reading (thread-safe)."""
        with self._imu_lock:
            if not self._imu_data:
                return None
            return dict(self._imu_data)

    def update_target(self, ip, port):
        """Update ESP32 IP and port at runtime."""
        self.esp32_ip = ip
        self.udp_port = port
        self._connected = False
        self._last_ack_time = 0.0
        self._last_imu_time = 0.0
        self.imu_active = False
        log.info(f"ESP32 target updated: {ip}:{port}")

    def is_connected(self):
        """Check if ESP32 has responded recently."""
        return self._connected