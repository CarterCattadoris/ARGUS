#!/usr/bin/env python3
"""
ARGUS — Autonomous RC Vehicle Ground Control Station
Main application entry point.

Flask + Flask-SocketIO server serving the dashboard UI,
handling WebRTC signaling for video, and coordinating
the CV pipeline, PID autopilot, and ESP32 comms.
"""

import time
import json
import threading
import logging
import numpy as np
from flask import Flask, render_template, Response, request
from flask_socketio import SocketIO, emit

from cv_pipeline import CVPipeline
from pid_controller import PIDAutopilot
from comms import ESP32Comms, find_esp32_by_mac
from config import Config
from astar import find_path_world, find_path_along_waypoints, is_path_blocked

# ──────────────────────────────────────────────
# Logging
# ──────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
)
log = logging.getLogger("argus")

# ──────────────────────────────────────────────
# Flask + SocketIO
# ──────────────────────────────────────────────
app = Flask(__name__, template_folder='../templates', static_folder='../static')
app.config["SECRET_KEY"] = Config.SECRET_KEY

socketio = SocketIO(
    app,
    async_mode="threading",
    cors_allowed_origins="*",
    ping_interval=10,
    ping_timeout=20,
)

# ──────────────────────────────────────────────
# Subsystems
# ──────────────────────────────────────────────
cv_pipeline = CVPipeline(
    camera_index=Config.CAMERA_INDEX,
    resolution=Config.CAMERA_RESOLUTION,
    model_path=Config.YOLO_MODEL_PATH,
    confidence=Config.YOLO_CONFIDENCE,
)

# Auto-discover ESP32 by MAC address
_discovered_ip = find_esp32_by_mac(Config.ESP32_MAC)
_esp32_ip = _discovered_ip if _discovered_ip else Config.ESP32_IP
if _discovered_ip:
    log.info(f"ESP32 auto-discovered at {_discovered_ip}")
else:
    log.warning(f"ESP32 not found by MAC, using fallback IP: {Config.ESP32_IP}")

esp32 = ESP32Comms(
    esp32_ip=_esp32_ip,
    udp_port=Config.UDP_PORT,
    imu_port=Config.IMU_PORT,
    failsafe_timeout=Config.FAILSAFE_TIMEOUT_MS,
)

autopilot = PIDAutopilot(
    kp=Config.PID_KP,
    ki=Config.PID_KI,
    kd=Config.PID_KD,
    throttle_kp=Config.THROTTLE_KP,
    throttle_ki=Config.THROTTLE_KI,
    throttle_kd=Config.THROTTLE_KD,
)

# Shared state
state = {
    "autopilot_engaged": False,
    "target": None,          # (x_m, y_m) in world coords
    "waypoints": [],         # list of (x_m, y_m) for drawn paths
    "initial_waypoints": [], # memory of the full initial drawn path for progress UI
    "manual_command": None,  # latest manual drive command
    "astar_waypoints": [],   # auto-generated avoidance waypoints
}
state_lock = threading.Lock()

# Calibration state (accessed by control loop thread)
cal_lock = threading.Lock()
cal_state = {
    "imu_calibrating": False,
    "imu_samples": [],
    "imu_start_time": 0.0,
    "gyro_bias": 0.0,            # subtracted from gyro_z each frame
    "motor_calibrating": False,
    "motor_samples": [],
    "motor_start_time": 0.0,
    "motor_trim": 0.0,           # added to left, subtracted from right
}


# ──────────────────────────────────────────────
# Routes
# ──────────────────────────────────────────────
@app.route("/")
def index():
    return render_template("index.html")


@app.route("/video_feed")
def video_feed():
    """MJPEG fallback stream (used if WebRTC negotiation fails)."""
    def generate():
        while True:
            frame_bytes = cv_pipeline.get_jpeg_frame()
            if frame_bytes is not None:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + frame_bytes
                    + b"\r\n"
                )
            time.sleep(0.033) # rate limit to ~30 FPS

    return Response(
        generate(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )



# ──────────────────────────────────────────────
# Dashboard SocketIO Events
# ──────────────────────────────────────────────
@socketio.on("connect")
def on_connect():
    log.info(f"Client connected: {request.sid}")
    emit("connection_status", {
        "esp32": esp32.is_connected(),
        "camera": cv_pipeline.is_camera_open(),
        "imu": esp32.imu_active,
    })
    emit("esp32_config", {
        "ip": esp32.esp32_ip,
        "port": esp32.udp_port,
    })


@socketio.on("disconnect")
def on_disconnect():
    log.info(f"Client disconnected: {request.sid}")
    with state_lock:
        state["manual_command"] = None
        if state["autopilot_engaged"]:
            state["autopilot_engaged"] = False
            esp32.send_motor_command(0.0, 0.0)
            log.warning("Autopilot disengaged — client disconnected")


@socketio.on("manual_drive")
def handle_manual_drive(data):
    """
    Manual WASD drive commands from the dashboard.
    data: { "left": -1.0 to 1.0, "right": -1.0 to 1.0 }
    Positive = forward, negative = reverse per motor.
    """
    with state_lock:
        if state["autopilot_engaged"]:
            emit("error", {"msg": "Disengage autopilot before manual control"})
            return
        state["manual_command"] = data

    left = float(data.get("left", 0))
    right = float(data.get("right", 0))
    esp32.send_motor_command(left, right)


@socketio.on("manual_stop")
def handle_manual_stop():
    """Stop motors when keys released."""
    with state_lock:
        state["manual_command"] = None
    esp32.send_motor_command(0.0, 0.0)


@socketio.on("update_esp32_ip")
def handle_update_esp32_ip(data):
    """Update ESP32 IP and port from dashboard."""
    ip = data.get("ip", "").strip()
    port = data.get("port", Config.UDP_PORT)
    try:
        port = int(port)
    except (ValueError, TypeError):
        emit("error", {"msg": "Invalid port number"})
        return
    if not ip:
        emit("error", {"msg": "IP address cannot be empty"})
        return
    esp32.update_target(ip, port)
    log.info(f"ESP32 target updated from dashboard: {ip}:{port}")
    emit("esp32_config", {"ip": ip, "port": port})


@socketio.on("scan_esp32")
def handle_scan_esp32():
    """Scan network for ESP32 by MAC address."""
    log.info("Dashboard triggered ESP32 MAC scan")
    ip = find_esp32_by_mac(Config.ESP32_MAC)
    if ip:
        esp32.update_target(ip, esp32.udp_port)
        emit("esp32_config", {"ip": ip, "port": esp32.udp_port})
        emit("scan_result", {"success": True, "ip": ip})
    else:
        emit("scan_result", {"success": False})


@socketio.on("toggle_autopilot")
def handle_toggle_autopilot(data):
    engage = data.get("engage", False)
    with state_lock:
        state["autopilot_engaged"] = engage
        if not engage:
            esp32.send_motor_command(0.0, 0.0)
            autopilot.reset()
            log.info("Autopilot disengaged")
        else:
            log.info("Autopilot engaged")

    emit("autopilot_state", {"engaged": engage})


@socketio.on("calibrate_imu")
def handle_calibrate_imu():
    """Start gyro bias calibration — car must be stationary."""
    log.info("IMU gyro bias calibration requested")
    with cal_lock:
        cal_state["imu_calibrating"] = True
        cal_state["imu_samples"] = []
        cal_state["imu_start_time"] = time.time()
    emit("calibration_status", {"type": "imu", "status": "started"})


@socketio.on("calibrate_motors")
def handle_calibrate_motors():
    """Start open-loop motor drift calibration and draw trace line."""
    log.info("Motor trim calibration requested")
    with cal_lock:
        cal_state["motor_calibrating"] = True
        cal_state["motor_samples"] = []
        cal_state["motor_start_time"] = 0.0  # 0.0 means uninitialized
    emit("calibration_status", {"type": "motors", "status": "started"})


@socketio.on("calibrate_floor")
def handle_calibrate_floor():
    """Capture floor colour profile for obstacle detection."""
    log.info("Floor calibration requested from dashboard")
    success = cv_pipeline.calibrate_floor()
    if success:
        emit("floor_calibration_status", {"success": True})
        log.info("Floor calibration successful")
    else:
        emit("floor_calibration_status", {"success": False})
        log.warning("Floor calibration failed")


@socketio.on("set_target")
def handle_set_target(data):
    """
    Set docking target from dashboard click.
    Converts pixel coords to world coordinates via homography.
    """
    px, py = float(data["px"]), float(data["py"])
    world_coords = cv_pipeline.pixel_to_world(px, py)
    if world_coords is not None:
        with state_lock:
            state["target"] = world_coords
            state["waypoints"] = []
            state["initial_waypoints"] = []
            state["astar_waypoints"] = []
        log.info(f"Target set: pixel=({px:.0f},{py:.0f}) → world=({world_coords[0]:.3f},{world_coords[1]:.3f})m")
        emit("target_confirmed", {
            "world_x": world_coords[0],
            "world_y": world_coords[1],
            "px": px,
            "py": py,
        })
    else:
        emit("error", {"msg": "Camera not calibrated — cannot map pixel to world coords"})


@socketio.on("set_waypoints")
def handle_set_waypoints(data):
    """
    Receive drawn path as list of pixel coordinates, convert to world coords.
    data: { "points": [ {"px": x, "py": y}, ... ] }
    """
    points = data.get("points", [])
    world_points = []
    for pt in points:
        wc = cv_pipeline.pixel_to_world(float(pt["px"]), float(pt["py"]))
        if wc is not None:
            world_points.append(wc)

    if len(world_points) < 2:
        emit("error", {"msg": "Need at least 2 valid waypoints"})
        return

    simplified = simplify_path(world_points, min_distance=0.03)

    with state_lock:
        state["waypoints"] = simplified
        state["initial_waypoints"] = list(simplified)
        state["target"] = simplified[-1]
        state["astar_waypoints"] = []

    log.info(f"Path set: {len(simplified)} waypoints, final target=({simplified[-1][0]:.3f},{simplified[-1][1]:.3f})m")
    emit("waypoints_confirmed", {
        "count": len(simplified),
        "points": [{"x": p[0], "y": p[1]} for p in simplified],
    })


# ──────────────────────────────────────────────
# Path simplification
# ──────────────────────────────────────────────
def simplify_path(points, min_distance=0.03):
    """Remove points that are too close together."""
    if len(points) < 2:
        return points
    simplified = [points[0]]
    for pt in points[1:]:
        dx = pt[0] - simplified[-1][0]
        dy = pt[1] - simplified[-1][1]
        if (dx * dx + dy * dy) ** 0.5 >= min_distance:
            simplified.append(pt)
    if simplified[-1] != points[-1]:
        simplified.append(points[-1])
    return simplified


# ──────────────────────────────────────────────
# Main Control Loop (runs in background thread)
# ──────────────────────────────────────────────
def control_loop():
    """
    ~20Hz loop: read CV detections + IMU, run PID if autopilot
    engaged, send motor commands, emit telemetry to dashboard.
    """
    log.info("Control loop started")
    loop_rate = 1.0 / Config.CONTROL_LOOP_HZ
    fps_counter = 0
    fps_time = time.time()
    current_fps = 0.0

    # IMU integration globals
    global_yaw = None
    last_yaw_time = time.time()

    # A* replanning throttle
    last_replan_time = 0.0

    # Heading calibration spin state
    heading_cal_active = False
    heading_cal_start_time = 0.0
    heading_cal_readings = []  # collect velocity_heading samples during spin

    while True:
        t_start = time.time()

        # 1) Get latest detection from CV pipeline
        detection = cv_pipeline.get_latest_detection()

        # 2) Get latest IMU data from ESP32
        imu_data = esp32.get_imu_data()

        # 3) Build current pose
        position = None
        speed = 0.0
        if detection:
            position = (detection["world_x"], detection["world_y"])
            speed = detection.get("speed", 0.0)

        # ── IMU gyro bias calibration ──
        with cal_lock:
            imu_calibrating = cal_state["imu_calibrating"]
            gyro_bias = cal_state["gyro_bias"]
            motor_calibrating = cal_state["motor_calibrating"]
            motor_trim = cal_state["motor_trim"]

        if imu_calibrating and imu_data and imu_data.get("gyro_z") is not None:
            with cal_lock:
                cal_state["imu_samples"].append(imu_data["gyro_z"])
                elapsed_cal = t_start - cal_state["imu_start_time"]
                if elapsed_cal >= 2.0:  # collect for 2 seconds
                    samples = cal_state["imu_samples"]
                    cal_state["gyro_bias"] = float(np.mean(samples))
                    cal_state["imu_calibrating"] = False
                    gyro_bias = cal_state["gyro_bias"]
                    log.info(f"IMU calibrated: gyro_z bias = {gyro_bias:.4f}°/s from {len(samples)} samples")
                    socketio.emit("calibration_status", {
                        "type": "imu", "status": "done",
                        "bias": round(gyro_bias, 4)
                    })

        # 4) Fuse heading: IMU primary, CV velocity vector as drift correction
        heading = None
        
        # Integrate ESP32 Gyro Z for relative heading (with bias correction)
        if imu_data and imu_data.get("gyro_z") is not None:
            dt_imu = t_start - last_yaw_time
            gz = imu_data["gyro_z"] - gyro_bias  # subtract calibrated bias
            if global_yaw is None:
                if detection and detection.get("velocity_heading") is not None:
                    global_yaw = detection["velocity_heading"]
                else:
                    global_yaw = 0.0
            global_yaw = (global_yaw - gz * dt_imu) % 360
        last_yaw_time = t_start

        # Break drift using CV vector
        if detection and detection.get("velocity_heading") is not None:
            cv_heading = detection["velocity_heading"]
            if global_yaw is not None and speed > 0.05:
                diff = (cv_heading - global_yaw + 180) % 360 - 180
                global_yaw = (global_yaw + 0.05 * diff) % 360
            elif global_yaw is None:
                global_yaw = cv_heading
                
        heading = global_yaw

        # ── Motor open-loop trim calibration ──
        with cal_lock:
            motor_calibrating = cal_state.get("motor_calibrating", False)
            
        if motor_calibrating:
            with cal_lock:
                start_t = cal_state.get("motor_start_time", 0.0)
                if start_t == 0.0:
                    cal_state["motor_start_time"] = t_start
                    # Draw a 1.5m straight line forward for visual reference
                    if position and heading is not None:
                        import math
                        rad = math.radians(heading)
                        target_x = position[0] + math.sin(rad) * 1.5
                        target_y = position[1] - math.cos(rad) * 1.5
                        with state_lock:
                            state["waypoints"] = [position, (target_x, target_y)]
                            state["target"] = (target_x, target_y)
                            state["astar_waypoints"] = []
                        socketio.emit("waypoints_confirmed", {
                            "count": 2,
                            "points": [{"x": p[0], "y": p[1]} for p in state["waypoints"]]
                        })
                
                elapsed = t_start - cal_state["motor_start_time"]
                
                # Active driving phase
                if elapsed < 2.5:
                    if imu_data and imu_data.get("gyro_z") is not None:
                        cal_state["motor_samples"].append(imu_data["gyro_z"] - gyro_bias)
                    telem_left, telem_right = 0.7, 0.7
                    esp32.send_motor_command(0.7, 0.7)
                else:
                    esp32.send_motor_command(0.0, 0.0)
                    cal_state["motor_calibrating"] = False
                    samples = cal_state["motor_samples"]
                    if samples:
                        trim = (sum(samples) / len(samples)) * 0.005
                        trim = max(-0.15, min(0.15, trim))
                        cal_state["motor_trim"] = trim
                        log.info(f"Motors calibrated: trim={trim:.3f}")
                    else:
                        trim = cal_state["motor_trim"]
                    socketio.emit("calibration_status", {
                         "type": "motors", "status": "done", "trim": round(trim, 3)
                    })

        # 5) Autopilot logic
        telem_left = 0.0
        telem_right = 0.0

        with state_lock:
            ap_engaged = state["autopilot_engaged"] and not motor_calibrating
            target = state["target"]
            waypoints = list(state["waypoints"])
            initial_waypoints = list(state["initial_waypoints"])
            astar_waypoints = list(state["astar_waypoints"])
            
        cv_pipeline.set_nav_data(waypoints, target, initial_waypoints)

        # ── A* obstacle avoidance replanning ──
        # Only replan if: we have no path, OR the current path is blocked
        if (ap_engaged and position is not None and target is not None
                and cv_pipeline.is_floor_calibrated()
                and t_start - last_replan_time >= Config.OBSTACLE_REPLAN_INTERVAL):
            last_replan_time = t_start
            grid = cv_pipeline.get_occupancy_grid()
            detector = cv_pipeline.get_obstacle_detector()
            if grid is not None and detector is not None:
                needs_replan = (
                    not astar_waypoints or  # no path yet
                    is_path_blocked(grid, astar_waypoints, detector)  # current path hits obstacle
                )
                if needs_replan:
                    if waypoints:
                        # User drew a path — route A* along their waypoints,
                        # only detouring around obstacles on blocked segments
                        path = find_path_along_waypoints(
                            grid, position, waypoints, target, detector
                        )
                    else:
                        # No drawn path — find shortest route to target
                        path = find_path_world(grid, position, target, detector)

                    if path and len(path) >= 2:
                        with state_lock:
                            state["astar_waypoints"] = path
                        astar_waypoints = path
                        cv_pipeline.set_astar_path(path)
                    else:
                        with state_lock:
                            state["astar_waypoints"] = []
                        astar_waypoints = []
                        cv_pipeline.set_astar_path([])

        # Determine which waypoints to follow (A* takes priority when available)
        active_waypoints = astar_waypoints if astar_waypoints else waypoints

        if ap_engaged and position is not None and target is not None:
            # ── Heading calibration spin ──
            # On first autopilot engage with unknown heading, do a slow
            # in-place rotation so the CV pipeline can build up velocity
            # history and compute a reliable heading.
            if global_yaw is None and not heading_cal_active:
                heading_cal_active = True
                heading_cal_start_time = t_start
                heading_cal_readings = []
                log.info("Heading calibration: starting spin...")

            if heading_cal_active:
                elapsed_cal = t_start - heading_cal_start_time

                # Collect any velocity heading samples
                if detection and detection.get("velocity_heading") is not None:
                    heading_cal_readings.append(detection["velocity_heading"])

                # End conditions: got enough readings OR time expired
                got_readings = len(heading_cal_readings) >= 3
                timed_out = elapsed_cal >= Config.HEADING_CAL_DURATION

                if got_readings or timed_out:
                    if heading_cal_readings:
                        # Use the median of collected headings for robustness
                        cal_heading = float(np.median(heading_cal_readings))
                        global_yaw = cal_heading
                        heading = global_yaw
                        log.info(f"Heading calibrated: {cal_heading:.1f}° from {len(heading_cal_readings)} samples")
                    else:
                        global_yaw = 0.0
                        heading = 0.0
                        log.warning("Heading calibration timed out with no CV heading — defaulting to 0°")

                    heading_cal_active = False
                    esp32.send_motor_command(0.0, 0.0)  # stop spin
                    time.sleep(0.1)  # brief pause before nav starts
                else:
                    # Still calibrating — send spin command
                    spin_spd = Config.HEADING_CAL_SPIN_SPEED
                    esp32.send_motor_command(-spin_spd, spin_spd)  # in-place CW rotation
                    telem_left = -spin_spd
                    telem_right = spin_spd

            elif heading is not None:
                # Normal autopilot navigation
                # Determine current waypoint target
                if active_waypoints:
                    # ── Dynamic Pruner: drop any raw waypoints the robot has already driven past
                    with state_lock:
                        if state["waypoints"]:
                            wps = state["waypoints"]
                            d0 = (position[0] - wps[0][0])**2 + (position[1] - wps[0][1])**2
                            
                            # If we reached it, or are closer to the next one, pop the stale one
                            if d0**0.5 < Config.WAYPOINT_REACH_THRESHOLD:
                                state["waypoints"].pop(0)
                            elif len(wps) > 1:
                                d1 = (position[0] - wps[1][0])**2 + (position[1] - wps[1][1])**2
                                if d1 < d0:
                                    state["waypoints"].pop(0)

                    current_target = active_waypoints[0]
                    dx = current_target[0] - position[0]
                    dy = current_target[1] - position[1]
                    dist = (dx * dx + dy * dy) ** 0.5
                    # A* waypoints use a larger reach threshold
                    reach = 0.16 if astar_waypoints else Config.WAYPOINT_REACH_THRESHOLD
                    if dist < reach:
                        if astar_waypoints:
                            with state_lock:
                                if state["astar_waypoints"]:
                                    state["astar_waypoints"].pop(0)
                                    if state["astar_waypoints"]:
                                        current_target = state["astar_waypoints"][0]
                                    else:
                                        state["waypoints"] = []
                                        current_target = target
                        else:
                            with state_lock:
                                if state["waypoints"]:
                                    state["waypoints"].pop(0)
                                    if state["waypoints"]:
                                        current_target = state["waypoints"][0]
                                    else:
                                        current_target = target
                else:
                    current_target = target

                pid_output = autopilot.compute(
                    position=position,
                    heading=heading,
                    target=current_target,
                    dt=loop_rate,
                )

                telem_left = pid_output["left"]
                telem_right = pid_output["right"]
                esp32.send_motor_command(telem_left, telem_right)

        else:
            if state["manual_command"] is None and not cal_state.get("motor_calibrating", False):
                esp32.send_motor_command(0.0, 0.0)

        # 6) Compute distance to target
        distance_to_target = None
        if position and target:
            dx = target[0] - position[0]
            dy = target[1] - position[1]
            distance_to_target = (dx * dx + dy * dy) ** 0.5

        # 7) FPS counter
        fps_counter += 1
        if time.time() - fps_time >= 1.0:
            current_fps = fps_counter
            fps_counter = 0
            fps_time = time.time()

        # 8) Emit telemetry to dashboard
        telemetry = {
            "timestamp": time.time(),
            "position": {"x": position[0], "y": position[1]} if position else None,
            "heading": round(heading, 1) if heading is not None else None,
            "speed": round(speed, 3),
            "distance_to_target": round(distance_to_target, 3) if distance_to_target is not None else None,
            "motors": {
                "left": round(telem_left, 2),
                "right": round(telem_right, 2),
            },
            "pid": {
                "kp": autopilot.steering_pid.kp,
                "ki": autopilot.steering_pid.ki,
                "kd": autopilot.steering_pid.kd,
                "steering_error": round(autopilot.last_steering_error, 3) if autopilot.last_steering_error else 0,
                "throttle_error": round(autopilot.last_throttle_error, 3) if autopilot.last_throttle_error else 0,
            },
            "autopilot": ap_engaged,
            "target": {"x": target[0], "y": target[1]} if target else None,
            "imu": {
                "accel_x": round(imu_data.get("accel_x"), 3) if imu_data and imu_data.get("accel_x") is not None else None,
                "accel_y": round(imu_data.get("accel_y"), 3) if imu_data and imu_data.get("accel_y") is not None else None,
                "accel_z": round(imu_data.get("accel_z"), 3) if imu_data and imu_data.get("accel_z") is not None else None,
                "gyro_x": round(imu_data.get("gyro_x"), 2) if imu_data and imu_data.get("gyro_x") is not None else None,
                "gyro_y": round(imu_data.get("gyro_y"), 2) if imu_data and imu_data.get("gyro_y") is not None else None,
                "gyro_z": round(imu_data.get("gyro_z"), 2) if imu_data and imu_data.get("gyro_z") is not None else None,
            },
            "cv_metrics": {
                "cv_fps": round(cv_pipeline.metrics["cv_fps"], 1),
                "inference_ms": round(cv_pipeline.metrics["inference_ms"], 1),
                "other_ms": round(cv_pipeline.metrics["other_ms"], 1),
            },
            "fps": current_fps,
            "connection": {
                "esp32": esp32.is_connected(),
                "camera": cv_pipeline.is_camera_open(),
                "imu": esp32.imu_active,
            },
            "obstacles": {
                "calibrated": cv_pipeline.is_floor_calibrated(),
                "count": cv_pipeline.get_obstacle_count(),
            },
        }

        socketio.emit("telemetry", telemetry)

        # 9) Sleep to maintain loop rate
        elapsed = time.time() - t_start
        sleep_time = loop_rate - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


# ──────────────────────────────────────────────
# Startup
# ──────────────────────────────────────────────
def start_subsystems():
    log.info("=" * 50)
    log.info("  ARGUS Ground Control Station")
    log.info("=" * 50)

    cv_pipeline.start()
    log.info(f"CV pipeline started (camera={Config.CAMERA_INDEX}, "
             f"res={Config.CAMERA_RESOLUTION}, model={Config.YOLO_MODEL_PATH})")

    esp32.start()
    log.info(f"ESP32 comms started (ip={Config.ESP32_IP}, port={Config.UDP_PORT})")

    ctrl_thread = threading.Thread(target=control_loop, daemon=True)
    ctrl_thread.start()
    log.info(f"Control loop running at {Config.CONTROL_LOOP_HZ} Hz")

    log.info(f"Dashboard: http://0.0.0.0:{Config.FLASK_PORT}")
    log.info("=" * 50)


if __name__ == "__main__":
    start_subsystems()
    socketio.run(
        app,
        host="0.0.0.0",
        port=Config.FLASK_PORT,
        debug=False,
        allow_unsafe_werkzeug=True,
    )