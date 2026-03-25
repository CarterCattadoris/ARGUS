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
from flask import Flask, render_template, Response, request
from flask_socketio import SocketIO, emit

from cv_pipeline import CVPipeline
from pid_controller import PIDAutopilot
from comms import ESP32Comms
from config import Config

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

esp32 = ESP32Comms(
    esp32_ip=Config.ESP32_IP,
    udp_port=Config.UDP_PORT,
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
    "manual_command": None,  # latest manual drive command
}
state_lock = threading.Lock()


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
            else:
                time.sleep(0.03)

    return Response(
        generate(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


# ──────────────────────────────────────────────
# WebRTC Signaling (via SocketIO)
# ──────────────────────────────────────────────
@socketio.on("webrtc_offer")
def handle_webrtc_offer(data):
    """
    Receive SDP offer from browser, pass to CV pipeline's
    WebRTC handler, return SDP answer.
    """
    log.info("WebRTC offer received from client")
    answer = cv_pipeline.handle_webrtc_offer(data)
    if answer:
        emit("webrtc_answer", answer)
    else:
        emit("webrtc_error", {"error": "Failed to create WebRTC answer"})


@socketio.on("webrtc_ice_candidate")
def handle_ice_candidate(data):
    """Forward ICE candidate to the CV pipeline's peer connection."""
    cv_pipeline.add_ice_candidate(data)


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


@socketio.on("disconnect")
def on_disconnect():
    log.info(f"Client disconnected: {request.sid}")
    with state_lock:
        state["manual_command"] = None
        if state["autopilot_engaged"]:
            state["autopilot_engaged"] = False
            esp32.send_motor_command(0.0, 0.0, "fwd")
            log.warning("Autopilot disengaged — client disconnected")


@socketio.on("manual_drive")
def handle_manual_drive(data):
    """
    Manual WASD drive commands from the dashboard.
    data: { "left": 0.0-1.0, "right": 0.0-1.0, "dir": "fwd"/"rev" }
    """
    with state_lock:
        if state["autopilot_engaged"]:
            emit("error", {"msg": "Disengage autopilot before manual control"})
            return
        state["manual_command"] = data

    left = float(data.get("left", 0))
    right = float(data.get("right", 0))
    direction = data.get("dir", "fwd")
    esp32.send_motor_command(left, right, direction)


@socketio.on("manual_stop")
def handle_manual_stop():
    """Stop motors when keys released."""
    with state_lock:
        state["manual_command"] = None
    esp32.send_motor_command(0.0, 0.0, "fwd")


@socketio.on("toggle_autopilot")
def handle_toggle_autopilot(data):
    engage = data.get("engage", False)
    with state_lock:
        if engage and state["target"] is None and len(state["waypoints"]) == 0:
            emit("error", {"msg": "Set a target or draw a path before engaging autopilot"})
            return

        state["autopilot_engaged"] = engage
        if not engage:
            esp32.send_motor_command(0.0, 0.0, "fwd")
            autopilot.reset()
            log.info("Autopilot disengaged")
        else:
            log.info("Autopilot engaged")

    emit("autopilot_state", {"engaged": engage})


@socketio.on("set_target")
def handle_set_target(data):
    """
    Set docking target from dashboard click.
    data: { "px": pixel_x, "py": pixel_y }
    Converts pixel coords to world coordinates via the CV pipeline's
    homography matrix.
    """
    px, py = float(data["px"]), float(data["py"])
    world_coords = cv_pipeline.pixel_to_world(px, py)
    if world_coords is not None:
        with state_lock:
            state["target"] = world_coords
            state["waypoints"] = []  # clear drawn path when setting single target
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

    # Simplify path — keep every Nth point to reduce noise
    simplified = simplify_path(world_points, min_distance=0.03)  # 3cm min spacing

    with state_lock:
        state["waypoints"] = simplified
        state["target"] = simplified[-1]  # final waypoint is the target

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
    # Always include the last point
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

    while True:
        t_start = time.time()

        # 1) Get latest detection from CV pipeline
        detection = cv_pipeline.get_latest_detection()

        # 2) Get latest IMU data from ESP32
        imu_data = esp32.get_imu_data()

        # 3) Fuse heading: IMU primary, CV velocity vector as drift correction
        heading = None
        if imu_data and imu_data.get("yaw") is not None:
            heading = imu_data["yaw"]
        if detection and detection.get("velocity_heading") is not None:
            if heading is not None:
                # Complementary filter: 90% IMU, 10% CV
                cv_heading = detection["velocity_heading"]
                heading = 0.9 * heading + 0.1 * cv_heading
            else:
                heading = detection["velocity_heading"]

        # 4) Build current pose
        position = None
        speed = 0.0
        if detection:
            position = (detection["world_x"], detection["world_y"])
            speed = detection.get("speed", 0.0)

        # 5) Autopilot logic
        left_power = 0.0
        right_power = 0.0
        direction = "fwd"
        pid_output = None

        with state_lock:
            ap_engaged = state["autopilot_engaged"]
            target = state["target"]
            waypoints = list(state["waypoints"])

        if ap_engaged and position is not None and heading is not None and target is not None:
            # Determine current waypoint target
            if waypoints:
                current_target = waypoints[0]
                # Check if we've reached this waypoint
                dx = current_target[0] - position[0]
                dy = current_target[1] - position[1]
                dist = (dx * dx + dy * dy) ** 0.5
                if dist < Config.WAYPOINT_REACH_THRESHOLD:
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

            left_power = pid_output["left"]
            right_power = pid_output["right"]
            direction = pid_output["dir"]
            esp32.send_motor_command(left_power, right_power, direction)

        elif not ap_engaged:
            # Manual mode — commands sent directly in handle_manual_drive
            pass

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
                "left": round(left_power, 2),
                "right": round(right_power, 2),
                "dir": direction,
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
                "yaw": imu_data.get("yaw") if imu_data else None,
                "accel_x": imu_data.get("accel_x") if imu_data else None,
                "accel_y": imu_data.get("accel_y") if imu_data else None,
            },
            "fps": current_fps,
            "connection": {
                "esp32": esp32.is_connected(),
                "camera": cv_pipeline.is_camera_open(),
                "imu": esp32.imu_active,
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
    """Initialize all subsystems before starting the control loop."""
    log.info("=" * 50)
    log.info("  ARGUS Ground Control Station")
    log.info("=" * 50)

    # Start camera + CV pipeline
    cv_pipeline.start()
    log.info(f"CV pipeline started (camera={Config.CAMERA_INDEX}, "
             f"res={Config.CAMERA_RESOLUTION}, model={Config.YOLO_MODEL_PATH})")

    # Start ESP32 comms (UDP listener for IMU data)
    esp32.start()
    log.info(f"ESP32 comms started (ip={Config.ESP32_IP}, port={Config.UDP_PORT})")

    # Start control loop in background thread
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
