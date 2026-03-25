"""
ARGUS Configuration
All tunable parameters in one place.
"""


class Config:
    # ── Flask ──────────────────────────────────
    SECRET_KEY = "argus-gcs-2026"
    FLASK_PORT = 5000

    # ── Camera ─────────────────────────────────
    CAMERA_INDEX = 0                     # /dev/video0
    CAMERA_RESOLUTION = (640, 480)     # 1080p capture
    STREAM_RESOLUTION = (960, 540)       # downscale for streaming
    JPEG_QUALITY = 70                    # MJPEG fallback quality

    # ── YOLO Detection ─────────────────────────
    YOLO_MODEL_PATH = "models/best.pt"   # path to custom YOLOv11n weights
    YOLO_CONFIDENCE = 0.5                # min detection confidence
    YOLO_INPUT_SIZE = 416                # model input resolution
    YOLO_CLASS_NAME = "rc_car"           # expected class label

    # ── ArUco (fallback/calibration) ───────────
    ARUCO_DICT = "DICT_4X4_50"
    ARUCO_MARKER_SIZE_M = 0.05           # 5cm markers for calibration

    # ── Camera Calibration ─────────────────────
    CALIBRATION_FILE = "calibration/camera_params.npz"
    HOMOGRAPHY_FILE = "calibration/homography.npz"

    # ── ESP32 Communication ────────────────────
    ESP32_IP = "10.144.113.116"           # ← change to your ESP32's static IP
    UDP_PORT = 4210                      # bidirectional: cmds out, IMU in
    FAILSAFE_TIMEOUT_MS = 500            # motors stop if no cmd in 500ms
    IMU_TIMEOUT_S = 2.0                  # mark IMU inactive after 2s silence

    # ── Control Loop ───────────────────────────
    CONTROL_LOOP_HZ = 20                 # 20 Hz main loop

    # ── PID — Steering ─────────────────────────
    PID_KP = 1.20
    PID_KI = 0.05
    PID_KD = 0.30
    PID_OUTPUT_LIMIT = 1.0               # max correction magnitude

    # ── PID — Throttle ─────────────────────────
    THROTTLE_KP = 0.80
    THROTTLE_KI = 0.02
    THROTTLE_KD = 0.10
    BASE_THROTTLE = 0.45                 # cruising power
    MIN_THROTTLE = 0.20                  # min motor power to move
    MAX_THROTTLE = 0.85                  # safety cap
    APPROACH_SLOW_DIST = 0.25            # slow down within 25cm of target

    # ── Navigation ─────────────────────────────
    WAYPOINT_REACH_THRESHOLD = 0.08      # 8cm — waypoint "reached"
    DOCKING_THRESHOLD = 0.04             # 4cm — final docking precision
    HEADING_TOLERANCE_DEG = 5.0          # degrees of acceptable heading error

    # ── Arena ──────────────────────────────────
    ARENA_WIDTH_M = 2.0
    ARENA_HEIGHT_M = 2.0

    # ── Tracking ───────────────────────────────
    POSITION_HISTORY_LENGTH = 50         # frames of position history for trail
    VELOCITY_WINDOW = 5                  # frames for velocity estimation
