"""
ARGUS — Computer Vision Pipeline
Handles camera capture, YOLOv11n detection, object tracking,
coordinate mapping (pixel → world), obstacle detection, and
video streaming.
"""

import cv2
import numpy as np
import time
import threading
import logging
import math
from collections import deque
from obstacle_detection import FloorCalibrator, ObstacleDetector

log = logging.getLogger("argus.cv")


class CVPipeline:
    def __init__(self, camera_index=0, resolution=(1920, 1080),
                 model_path="models/best.pt", confidence=0.5):
        self.camera_index = camera_index
        self.resolution = resolution
        self.model_path = model_path
        self.confidence = confidence

        # Camera
        self._cap = None
        self._cap_lock = threading.Lock()

        # YOLO model
        self._model = None

        # Latest frame + detection
        self._latest_frame = None
        self._latest_detection = None
        self._frame_lock = threading.Lock()

        # Position history for trail + velocity
        self._position_history = deque(maxlen=50)
        self._detection_times = deque(maxlen=50)

        # Coordinate mapping
        self._homography = None  # 3x3 pixel→world transform
        self._camera_matrix = None
        self._dist_coeffs = None


        # Obstacle detection
        self._floor_calibrator = FloorCalibrator()
        self._obstacle_detector = None  # initialised after calibration
        self._obstacle_frame_counter = 0
        self._astar_path_world = []     # latest A* path in world coords
        self._astar_path_lock = threading.Lock()

        # Thread control
        self._running = False
        self._capture_thread = None

    # ──────────────────────────────────────────
    # Startup
    # ──────────────────────────────────────────
    def start(self):
        """Open camera, load model, load calibration, init obstacle detector, start capture thread."""
        self._open_camera()
        self._load_model()
        self._load_calibration()

        # Initialise obstacle detector (needs pixel_to_world)
        self._obstacle_detector = ObstacleDetector(
            calibrator=self._floor_calibrator,
            pixel_to_world_fn=self.pixel_to_world,
        )

        self._running = True
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

    def stop(self):
        self._running = False
        if self._capture_thread:
            self._capture_thread.join(timeout=3)
        if self._cap:
            self._cap.release()

    # ──────────────────────────────────────────
    # Camera
    # ──────────────────────────────────────────
    def _open_camera(self):
        """Open USB webcam with V4L2 backend."""
        try:
            self._cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
            self._cap.set(cv2.CAP_PROP_FPS, 30)
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # minimize latency

            if not self._cap.isOpened():
                log.error("Failed to open camera %d", self.camera_index)
                self._cap = None
            else:
                actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                log.info(f"Camera opened: {actual_w}x{actual_h}")
        except Exception as e:
            log.error(f"Camera open error: {e}")
            self._cap = None

    def is_camera_open(self):
        return self._cap is not None and self._cap.isOpened()

    # ──────────────────────────────────────────
    # YOLO Model
    # ──────────────────────────────────────────
    def _load_model(self):
        """Load YOLOv11n model via Ultralytics."""
        try:
            from ultralytics import YOLO
            self._model = YOLO(self.model_path)
            log.info(f"YOLO model loaded: {self.model_path}")
        except FileNotFoundError:
            log.warning(f"Model not found at {self.model_path} — detection disabled")
            self._model = None
        except Exception as e:
            log.error(f"Failed to load YOLO model: {e}")
            self._model = None

    # ──────────────────────────────────────────
    # Calibration
    # ──────────────────────────────────────────
    def _load_calibration(self):
        """Load camera intrinsics and homography from disk."""
        from config import Config

        # Camera intrinsics
        try:
            data = np.load(Config.CALIBRATION_FILE)
            self._camera_matrix = data["camera_matrix"]
            self._dist_coeffs = data["dist_coeffs"]
            log.info("Camera calibration loaded")
        except FileNotFoundError:
            log.warning("No camera calibration found — using uncalibrated mode")

        # Homography (pixel → world)
        try:
            data = np.load(Config.HOMOGRAPHY_FILE)
            self._homography = data["homography"]
            log.info("Homography matrix loaded")
        except FileNotFoundError:
            log.warning("No homography found — using simple linear mapping instead of perspective correction.")

    def pixel_to_world(self, px, py):
        """
        Convert pixel coordinates to world coordinates (meters).
        Uses homography if available, otherwise falls back to a simple linear mapping.
        """
        if self._homography is not None:
            pt = np.array([[[px, py]]], dtype=np.float64)
            world = cv2.perspectiveTransform(pt, self._homography)
            return (float(world[0][0][0]), float(world[0][0][1]))
        else:
            # Fallback simple linear mapping
            from config import Config
            w, h = Config.CAMERA_RESOLUTION
            x_m = (px / w) * Config.ARENA_WIDTH_M
            y_m = (py / h) * Config.ARENA_HEIGHT_M
            return (float(x_m), float(y_m))

    def world_to_pixel(self, x_m, y_m):
        """Convert world coordinates back to pixel coordinates."""
        if self._homography is not None:
            try:
                inv_h = np.linalg.inv(self._homography)
                pt = np.array([[[x_m, y_m]]], dtype=np.float64)
                pixel = cv2.perspectiveTransform(pt, inv_h)
                return (float(pixel[0][0][0]), float(pixel[0][0][1]))
            except np.linalg.LinAlgError:
                pass
            
        # Fallback simple linear mapping
        from config import Config
        w, h = Config.CAMERA_RESOLUTION
        px = (x_m / Config.ARENA_WIDTH_M) * w
        py = (y_m / Config.ARENA_HEIGHT_M) * h
        return (float(px), float(py))

    # ──────────────────────────────────────────
    # Capture + Detection Loop
    # ──────────────────────────────────────────
    def _capture_loop(self):
        """Continuous capture → detect → track loop."""
        log.info("Capture loop started")

        while self._running:
            if not self.is_camera_open():
                time.sleep(0.5)
                self._open_camera()
                continue

            with self._cap_lock:
                ret, frame = self._cap.read()

            if not ret or frame is None:
                log.warning("Frame capture failed")
                time.sleep(0.03)
                continue

            # Undistort if calibrated
            if self._camera_matrix is not None and self._dist_coeffs is not None:
                frame = cv2.undistort(frame, self._camera_matrix, self._dist_coeffs)

            # Run YOLO detection
            detection = self._detect(frame)

            # Update tracking history
            t_now = time.time()
            if detection:
                self._position_history.append(
                    (detection["center_px"], detection["center_py"])
                )
                self._detection_times.append(t_now)

                # Estimate velocity from position history
                detection["speed"] = self._estimate_speed()
                detection["velocity_heading"] = self._estimate_velocity_heading()

            # Run obstacle detection periodically (not every frame for perf)
            from config import Config
            self._obstacle_frame_counter += 1
            if (self._floor_calibrator.calibrated and
                    self._obstacle_detector is not None and
                    self._obstacle_frame_counter % Config.OBSTACLE_DETECT_INTERVAL == 0):
                robot_bbox = detection["bbox"] if detection else None
                self._obstacle_detector.detect(frame, robot_bbox=robot_bbox)

            # Draw overlays on frame
            annotated = self._draw_overlays(frame, detection)

            # Store latest
            with self._frame_lock:
                self._latest_frame = annotated
                self._latest_detection = detection

        log.info("Capture loop stopped")

    def _detect(self, frame):
        """Run YOLOv11n inference on a frame."""
        if self._model is None:
            return None

        from config import Config

        results = self._model.predict(
            frame,
            imgsz=Config.YOLO_INPUT_SIZE,
            conf=self.confidence,
            verbose=False,
        )

        if not results or len(results[0].boxes) == 0:
            return None

        # Take highest-confidence detection
        boxes = results[0].boxes
        best_idx = boxes.conf.argmax().item()
        box = boxes.xyxy[best_idx].cpu().numpy()
        conf = float(boxes.conf[best_idx].cpu())
        cls_id = int(boxes.cls[best_idx].cpu())

        x1, y1, x2, y2 = box
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2

        # Convert to world coordinates
        world_x, world_y = None, None
        wc = self.pixel_to_world(cx, cy)
        if wc:
            world_x, world_y = wc

        return {
            "bbox": (float(x1), float(y1), float(x2), float(y2)),
            "center_px": float(cx),
            "center_py": float(cy),
            "confidence": conf,
            "class_id": cls_id,
            "world_x": world_x,
            "world_y": world_y,
            "speed": 0.0,
            "velocity_heading": None,
        }

    def _estimate_speed(self):
        """Estimate speed in m/s from recent position history."""
        from config import Config
        n = min(Config.VELOCITY_WINDOW, len(self._position_history))
        if n < 2:
            return 0.0

        positions = list(self._position_history)[-n:]
        times = list(self._detection_times)[-n:]

        # Convert pixel positions to world if possible
        world_positions = []
        for px, py in positions:
            wc = self.pixel_to_world(px, py)
            if wc:
                world_positions.append(wc)

        if len(world_positions) < 2:
            return 0.0

        total_dist = 0.0
        for i in range(1, len(world_positions)):
            dx = world_positions[i][0] - world_positions[i - 1][0]
            dy = world_positions[i][1] - world_positions[i - 1][1]
            total_dist += math.sqrt(dx * dx + dy * dy)

        dt = times[-1] - times[0]
        if dt <= 0:
            return 0.0

        return total_dist / dt

    def _estimate_velocity_heading(self):
        """Estimate heading from velocity vector (degrees, 0=north/up, CW)."""
        from config import Config
        n = min(Config.VELOCITY_WINDOW, len(self._position_history))
        if n < 2:
            return None

        positions = list(self._position_history)[-n:]
        world_positions = []
        for px, py in positions:
            wc = self.pixel_to_world(px, py)
            if wc:
                world_positions.append(wc)

        if len(world_positions) < 2:
            return None

        dx = world_positions[-1][0] - world_positions[0][0]
        dy = world_positions[-1][1] - world_positions[0][1]

        if abs(dx) < 0.005 and abs(dy) < 0.005:
            return None  # too slow to estimate

        heading = math.degrees(math.atan2(dx, -dy)) % 360
        return heading

    # ──────────────────────────────────────────
    # Overlays
    # ──────────────────────────────────────────
    def _draw_overlays(self, frame, detection):
        """Draw bounding box, trail, heading vector on frame."""
        annotated = frame.copy()

        # Draw position trail
        if len(self._position_history) > 1:
            pts = list(self._position_history)
            for i in range(1, len(pts)):
                alpha = i / len(pts)
                color = (int(37 * alpha), int(99 * alpha), int(235 * alpha))  # blue gradient
                thickness = max(1, int(2 * alpha))
                cv2.line(
                    annotated,
                    (int(pts[i - 1][0]), int(pts[i - 1][1])),
                    (int(pts[i][0]), int(pts[i][1])),
                    color,
                    thickness,
                    cv2.LINE_AA,
                )

        if detection:
            x1, y1, x2, y2 = detection["bbox"]

            # Bounding box — corner brackets style
            w = x2 - x1
            h = y2 - y1
            corner_len = min(w, h) * 0.25
            color = (37, 99, 235)  # accent blue
            t = 4

            # Top-left
            cv2.line(annotated, (int(x1), int(y1)), (int(x1 + corner_len), int(y1)), color, t, cv2.LINE_AA)
            cv2.line(annotated, (int(x1), int(y1)), (int(x1), int(y1 + corner_len)), color, t, cv2.LINE_AA)
            # Top-right
            cv2.line(annotated, (int(x2), int(y1)), (int(x2 - corner_len), int(y1)), color, t, cv2.LINE_AA)
            cv2.line(annotated, (int(x2), int(y1)), (int(x2), int(y1 + corner_len)), color, t, cv2.LINE_AA)
            # Bottom-left
            cv2.line(annotated, (int(x1), int(y2)), (int(x1 + corner_len), int(y2)), color, t, cv2.LINE_AA)
            cv2.line(annotated, (int(x1), int(y2)), (int(x1), int(y2 - corner_len)), color, t, cv2.LINE_AA)
            # Bottom-right
            cv2.line(annotated, (int(x2), int(y2)), (int(x2 - corner_len), int(y2)), color, t, cv2.LINE_AA)
            cv2.line(annotated, (int(x2), int(y2)), (int(x2), int(y2 - corner_len)), color, t, cv2.LINE_AA)

            # Center dot
            cx, cy = int(detection["center_px"]), int(detection["center_py"])
            cv2.circle(annotated, (cx, cy), 4, color, -1, cv2.LINE_AA)

            # Confidence label
            label = f"{detection['confidence']:.0%}"
            cv2.putText(
                annotated, label,
                (int(x1), int(y1) - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA,
            )

        # Draw navigation path
        nav_pts = getattr(self, "_nav_waypoints", [])
        nav_tgt = getattr(self, "_nav_target", None)

        if nav_pts:
            hw_pts = []
            for wp in nav_pts:
                px, py = self.world_to_pixel(wp[0], wp[1])
                hw_pts.append((int(px), int(py)))
            
            for i in range(1, len(hw_pts)):
                cv2.line(annotated, hw_pts[i-1], hw_pts[i], (0, 255, 255), 2, cv2.LINE_AA)
            cv2.circle(annotated, hw_pts[-1], 6, (0, 255, 255), -1, cv2.LINE_AA)
        elif nav_tgt:
            px, py = self.world_to_pixel(nav_tgt[0], nav_tgt[1])
            cv2.circle(annotated, (int(px), int(py)), 6, (0, 255, 255), -1, cv2.LINE_AA)

        # Draw debris overlay (semi-transparent red contours)
        if self._floor_calibrator.calibrated and self._obstacle_detector is not None:
            debris_mask = self._obstacle_detector.get_debris_mask()
            if debris_mask is not None:
                # Find contours and draw them
                contours, _ = cv2.findContours(
                    debris_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )
                if contours:
                    overlay = annotated.copy()
                    cv2.drawContours(overlay, contours, -1, (0, 0, 220), cv2.FILLED)
                    cv2.addWeighted(overlay, 0.25, annotated, 0.75, 0, annotated)
                    cv2.drawContours(annotated, contours, -1, (0, 0, 220), 1, cv2.LINE_AA)

        # Draw A* avoidance path (green)
        with self._astar_path_lock:
            astar_pts = list(self._astar_path_world)
        if len(astar_pts) > 1:
            pixel_pts = []
            for wp in astar_pts:
                px, py = self.world_to_pixel(wp[0], wp[1])
                pixel_pts.append((int(px), int(py)))
            for i in range(1, len(pixel_pts)):
                cv2.line(annotated, pixel_pts[i - 1], pixel_pts[i],
                         (0, 220, 0), 2, cv2.LINE_AA)
            # Draw small circles at waypoints
            for pt in pixel_pts:
                cv2.circle(annotated, pt, 3, (0, 220, 0), -1, cv2.LINE_AA)

        return annotated

    # ──────────────────────────────────────────
    # Public accessors
    # ──────────────────────────────────────────
    def set_nav_data(self, waypoints, target):
        with self._frame_lock:
            self._nav_waypoints = list(waypoints) if waypoints else []
            self._nav_target = target

    def set_astar_path(self, path):
        """Set the A* avoidance path for overlay drawing."""
        with self._astar_path_lock:
            self._astar_path_world = list(path) if path else []

    def get_latest_detection(self):
        with self._frame_lock:
            return self._latest_detection

    def calibrate_floor(self):
        """
        Capture a snapshot of the current frame and calibrate floor colour.
        Returns True on success.
        """
        with self._frame_lock:
            frame = self._latest_frame

        if frame is None:
            log.warning("Cannot calibrate floor — no frame available")
            return False

        # Get robot bbox if available
        det = self.get_latest_detection()
        robot_bbox = det["bbox"] if det else None

        success = self._floor_calibrator.calibrate(frame, robot_bbox=robot_bbox)
        if success:
            log.info("Floor calibration successful")
        return success

    def get_occupancy_grid(self):
        """Return the current occupancy grid, or None if not available."""
        if self._obstacle_detector is None:
            return None
        return self._obstacle_detector.get_occupancy_grid()

    def get_obstacle_detector(self):
        """Return the ObstacleDetector instance."""
        return self._obstacle_detector

    def get_obstacle_count(self):
        """Return number of occupied cells."""
        if self._obstacle_detector is None:
            return 0
        return self._obstacle_detector.get_obstacle_count()

    def is_floor_calibrated(self):
        """Return whether the floor has been calibrated."""
        return self._floor_calibrator.calibrated

    def get_jpeg_frame(self):
        """Get latest annotated frame as JPEG bytes (MJPEG fallback)."""
        from config import Config
        with self._frame_lock:
            if self._latest_frame is None:
                return None
            frame = self._latest_frame

        # Downscale for streaming
        sw, sh = Config.STREAM_RESOLUTION
        small = cv2.resize(frame, (sw, sh), interpolation=cv2.INTER_CUBIC)
        ret, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, Config.JPEG_QUALITY])
        return buf.tobytes() if ret else None
