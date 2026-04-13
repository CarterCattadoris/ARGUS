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
    CAMERA_RESOLUTION = (640, 480)       # Light capture for RPi
    STREAM_RESOLUTION = (640, 480)       # match camera for no scaling
    JPEG_QUALITY = 80                    # good quality, faster encoding for streaming

    # ── YOLO Detection ─────────────────────────
    YOLO_MODEL_PATH = "../models/best_ncnn_model"   # path to custom YOLOv11n weights
    YOLO_CONFIDENCE = 0.5                # min detection confidence
    YOLO_INPUT_SIZE = 416                # model input resolution (must match training size for NCNN)
    YOLO_CLASS_NAME = "robot"           # expected class label

    # ── ArUco (fallback/calibration) ───────────
    ARUCO_DICT = "DICT_4X4_50"
    ARUCO_MARKER_SIZE_M = 0.05           # 5cm markers for calibration

    # ── Camera Calibration ─────────────────────
    CALIBRATION_FILE = "calibration/camera_params.npz"
    HOMOGRAPHY_FILE = "calibration/homography.npz"

    # ── ESP32 Communication ────────────────────
    ESP32_MAC = "ac:23:16:f2:b3:a6"          # ESP32 MAC for auto-discovery
    ESP32_IP = "10.144.113.148"              # fallback if MAC scan fails
    UDP_PORT = 9876                          # motor commands out
    IMU_PORT = 9877                          # IMU telemetry in (from ESP32)
    FAILSAFE_TIMEOUT_MS = 500               # motors stop if no cmd in 500ms
    IMU_TIMEOUT_S = 2.0                     # mark IMU inactive after 2s silence

    # ── Control Loop ───────────────────────────
    CONTROL_LOOP_HZ = 20                 # 20 Hz main loop

    # ── PID — Steering ─────────────────────────
    PID_KP = 1.00
    PID_KI = 0.05
    PID_KD = 0.15
    PID_OUTPUT_LIMIT = 1.0               # max correction magnitude

    # ── PID — Throttle ─────────────────────────
    THROTTLE_KP = 1.00
    THROTTLE_KI = 0.05
    THROTTLE_KD = 0.20
    BASE_THROTTLE = 0.70                 # cruising power
    MIN_THROTTLE = 0.45                  # min motor power to move
    MAX_THROTTLE = 1.00                  # safety cap
    APPROACH_SLOW_DIST = 0.25            # slow down within 25cm of target

    # ── Navigation ─────────────────────────────
    WAYPOINT_REACH_THRESHOLD = 0.12      # 12cm — waypoint "reached"
    DOCKING_THRESHOLD = 0.06             # 6cm — final docking precision
    HEADING_TOLERANCE_DEG = 5.0          # degrees of acceptable heading error
    HEADING_CAL_SPIN_SPEED = 0.60        # motor power during heading calibration spin
    HEADING_CAL_DURATION = 3.0           # seconds to spin for heading calibration

    # ── Arena ──────────────────────────────────
    ARENA_WIDTH_M = 2.007      # 79 inches
    ARENA_HEIGHT_M = 4.420     # 174 inches

    # ── Tracking ───────────────────────────────
    POSITION_HISTORY_LENGTH = 50         # frames of position history for trail
    VELOCITY_WINDOW = 5                  # frames for velocity estimation

    # ── Obstacle Avoidance ─────────────────────
    FLOOR_CHROMA_THRESHOLD = 6.0         # Mahalanobis distance² in Lab a*b* space (higher = less sensitive)
    GRID_CELL_SIZE_M = 0.05             # 5cm occupancy grid cells
    ROBOT_CLEARANCE_CELLS = 4           # inflate obstacles by 4 cells (~20cm) — keeps robot visible to YOLO
    OBSTACLE_REPLAN_INTERVAL = 2.0      # re-run A* at most once every 3 seconds to prevent control stuttering
    OBSTACLE_DETECT_INTERVAL = 3        # run obstacle detection every N capture frames
