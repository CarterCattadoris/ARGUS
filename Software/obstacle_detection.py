"""
ARGUS — Obstacle Detection via Floor Color Segmentation

Identifies obstacles ("debris") by comparing each frame against a calibrated
floor color profile.  Outputs a binary occupancy grid in world coordinates
for use by the A* path-planner.

Uses temporal smoothing to prevent frame-to-frame oscillation of borderline
pixels caused by camera noise.
"""

import cv2
import numpy as np
import threading
import logging
from collections import deque

log = logging.getLogger("argus.obstacle")


class FloorCalibrator:
    """
    Captures a reference frame and computes the dominant floor color in HSV.
    Uses K-Means clustering on the frame (with the robot bbox masked out)
    so the dominant cluster corresponds to the floor surface.
    """

    def __init__(self, hsv_tolerance=None):
        from config import Config
        self._hsv_tolerance = hsv_tolerance or Config.FLOOR_HSV_TOLERANCE
        self._floor_hsv = None          # (H, S, V) centre
        self._lower_bound = None        # HSV lower bound for inRange
        self._upper_bound = None        # HSV upper bound for inRange
        self._calibrated = False
        self._lock = threading.Lock()

    # ─────────────────────────────────────────────
    @property
    def calibrated(self):
        with self._lock:
            return self._calibrated

    @property
    def floor_hsv(self):
        with self._lock:
            return self._floor_hsv

    # ─────────────────────────────────────────────
    def calibrate(self, frame, robot_bbox=None):
        """
        Snapshot the floor colour from *frame*.

        Args:
            frame:      BGR image (numpy array).
            robot_bbox: Optional (x1, y1, x2, y2) to mask out the robot.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Build a mask that excludes the robot (if detected)
        mask = np.ones(hsv.shape[:2], dtype=np.uint8) * 255
        if robot_bbox is not None:
            x1, y1, x2, y2 = [int(v) for v in robot_bbox]
            # Expand bbox slightly to be safe
            pad = 20
            x1 = max(0, x1 - pad)
            y1 = max(0, y1 - pad)
            x2 = min(frame.shape[1], x2 + pad)
            y2 = min(frame.shape[0], y2 + pad)
            mask[y1:y2, x1:x2] = 0

        # Collect valid pixels
        pixels = hsv[mask > 0].reshape(-1, 3).astype(np.float32)

        if len(pixels) < 100:
            log.warning("Not enough pixels for floor calibration")
            return False

        # K-Means with k=3 — pick the largest cluster as floor
        k = min(3, len(pixels) // 50)
        k = max(1, k)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 1.0)
        _, labels, centres = cv2.kmeans(
            pixels, k, None, criteria, 5, cv2.KMEANS_PP_CENTERS
        )

        # Find largest cluster
        counts = np.bincount(labels.flatten(), minlength=k)
        dominant_idx = counts.argmax()
        floor_hsv = centres[dominant_idx].astype(np.uint8)

        # Compute bounds
        h_tol, s_tol, v_tol = self._hsv_tolerance
        lower = np.array([
            max(0, int(floor_hsv[0]) - h_tol),
            max(0, int(floor_hsv[1]) - s_tol),
            max(0, int(floor_hsv[2]) - v_tol),
        ], dtype=np.uint8)
        upper = np.array([
            min(179, int(floor_hsv[0]) + h_tol),
            min(255, int(floor_hsv[1]) + s_tol),
            min(255, int(floor_hsv[2]) + v_tol),
        ], dtype=np.uint8)

        with self._lock:
            self._floor_hsv = tuple(int(v) for v in floor_hsv)
            self._lower_bound = lower
            self._upper_bound = upper
            self._calibrated = True

        log.info(
            f"Floor calibrated: HSV=({self._floor_hsv}), "
            f"range=[{lower.tolist()} – {upper.tolist()}]"
        )
        return True

    # ─────────────────────────────────────────────
    def get_bounds(self):
        """Return (lower, upper) HSV bounds, or (None, None) if not calibrated."""
        with self._lock:
            if not self._calibrated:
                return None, None
            return self._lower_bound.copy(), self._upper_bound.copy()


class ObstacleDetector:
    """
    Generates a binary debris mask from a BGR frame using the calibrated
    floor colour, and projects it onto an occupancy grid in world coordinates.
    """

    def __init__(self, calibrator, pixel_to_world_fn, grid_cell_size_m=None,
                 arena_width_m=None, arena_height_m=None, clearance_cells=None):
        from config import Config

        self._calibrator = calibrator
        self._pixel_to_world = pixel_to_world_fn

        self._cell_size = grid_cell_size_m or Config.GRID_CELL_SIZE_M
        self._arena_w = arena_width_m or Config.ARENA_WIDTH_M
        self._arena_h = arena_height_m or Config.ARENA_HEIGHT_M
        self._clearance = clearance_cells if clearance_cells is not None else Config.ROBOT_CLEARANCE_CELLS

        # Grid dimensions
        self._grid_cols = int(np.ceil(self._arena_w / self._cell_size))
        self._grid_rows = int(np.ceil(self._arena_h / self._cell_size))

        # Temporal smoothing: keep last N masks, require consistent detection
        self._mask_history = deque(maxlen=5)  # last 5 frames
        self._smoothing_threshold = 0.6       # pixel must be debris in 60% of frames

        # Latest results
        self._debris_mask = None
        self._occupancy_grid = None     # 0 = free, 1 = obstacle
        self._obstacle_count = 0
        self._lock = threading.Lock()

    # ─────────────────────────────────────────────
    @property
    def grid_rows(self):
        return self._grid_rows

    @property
    def grid_cols(self):
        return self._grid_cols

    @property
    def cell_size(self):
        return self._cell_size

    # ─────────────────────────────────────────────
    def detect(self, frame, robot_bbox=None):
        """
        Process a frame to find debris.

        Args:
            frame:      BGR image.
            robot_bbox: (x1, y1, x2, y2) of the detected robot to exclude.

        Returns:
            True if detection ran (calibrator ready), False otherwise.
        """
        lower, upper = self._calibrator.get_bounds()
        if lower is None:
            return False

        # Blur to smooth out lighting gradients across the frame
        blurred = cv2.GaussianBlur(frame, (15, 15), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Floor mask: pixels within the calibrated range
        floor_mask = cv2.inRange(hsv, lower, upper)

        # Debris = NOT floor
        debris_mask = cv2.bitwise_not(floor_mask)

        # Mask out the robot bounding box (robot is not debris)
        if robot_bbox is not None:
            x1, y1, x2, y2 = [int(v) for v in robot_bbox]
            pad = 25
            x1 = max(0, x1 - pad)
            y1 = max(0, y1 - pad)
            x2 = min(frame.shape[1], x2 + pad)
            y2 = min(frame.shape[0], y2 + pad)
            debris_mask[y1:y2, x1:x2] = 0

        # Morphological cleanup: moderate erosion + dilate
        kernel_erode = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        kernel_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        debris_mask = cv2.erode(debris_mask, kernel_erode, iterations=2)
        debris_mask = cv2.dilate(debris_mask, kernel_dilate, iterations=1)

        # Remove very small contours (dust/scratches)
        contours, _ = cv2.findContours(debris_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = 300  # px² — tape dispenser at ~640x480 is ~500-1500px²
        for cnt in contours:
            if cv2.contourArea(cnt) < min_area:
                cv2.drawContours(debris_mask, [cnt], -1, 0, cv2.FILLED)

        # ── Temporal smoothing ──
        # Accumulate binary mask (0 or 1) and threshold across frames
        binary = (debris_mask > 0).astype(np.float32)
        self._mask_history.append(binary)

        if len(self._mask_history) >= 2:
            # Average across recent frames
            stacked = np.stack(self._mask_history, axis=0)
            avg = np.mean(stacked, axis=0)
            # Only keep pixels that are consistently debris
            debris_mask = (avg >= self._smoothing_threshold).astype(np.uint8) * 255
        # If only 1 frame so far, use it as-is (first detection)

        # Build occupancy grid by sampling the debris mask at grid resolution
        grid = np.zeros((self._grid_rows, self._grid_cols), dtype=np.uint8)
        h, w = frame.shape[:2]

        # For efficiency, resize the debris mask to grid resolution and threshold
        # We'll map each grid cell back to pixel space
        for r in range(self._grid_rows):
            for c in range(self._grid_cols):
                # World coordinate of cell centre
                wx = (c + 0.5) * self._cell_size
                wy = (r + 0.5) * self._cell_size

                # Convert world → pixel (inverse of pixel_to_world)
                # We use a simple linear mapping for the grid projection
                px = int((wx / self._arena_w) * w)
                py = int((wy / self._arena_h) * h)

                if 0 <= px < w and 0 <= py < h:
                    # Check a small region around the pixel
                    region_size = max(1, int(self._cell_size / self._arena_w * w * 0.5))
                    rx1 = max(0, px - region_size)
                    ry1 = max(0, py - region_size)
                    rx2 = min(w, px + region_size)
                    ry2 = min(h, py + region_size)
                    region = debris_mask[ry1:ry2, rx1:rx2]
                    # If >30% of the region is debris, mark cell as obstacle
                    if region.size > 0 and np.mean(region) > 76:  # 76/255 ≈ 30%
                        grid[r, c] = 1

        # Inflate obstacles for robot clearance
        if self._clearance > 0:
            ksize = int(2 * self._clearance + 1)
            inflate_kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (ksize, ksize)
            )
            grid = cv2.dilate(grid, inflate_kernel, iterations=1)

        obstacle_count = int(np.sum(grid))

        with self._lock:
            self._debris_mask = debris_mask
            self._occupancy_grid = grid
            self._obstacle_count = obstacle_count

        return True

    # ─────────────────────────────────────────────
    def get_debris_mask(self):
        """Return latest binary debris mask (pixel-space), or None."""
        with self._lock:
            return self._debris_mask.copy() if self._debris_mask is not None else None

    def get_occupancy_grid(self):
        """Return (rows, cols) numpy uint8 grid.  0=free, 1=obstacle."""
        with self._lock:
            if self._occupancy_grid is not None:
                return self._occupancy_grid.copy()
            return np.zeros((self._grid_rows, self._grid_cols), dtype=np.uint8)

    def get_obstacle_count(self):
        """Number of occupied cells in the grid."""
        with self._lock:
            return self._obstacle_count

    # ─────────────────────────────────────────────
    def world_to_grid(self, x_m, y_m):
        """Convert world coordinates to grid cell (row, col)."""
        col = int(x_m / self._cell_size)
        row = int(y_m / self._cell_size)
        col = max(0, min(self._grid_cols - 1, col))
        row = max(0, min(self._grid_rows - 1, row))
        return (row, col)

    def grid_to_world(self, row, col):
        """Convert grid cell to world coordinates (centre of cell)."""
        x_m = (col + 0.5) * self._cell_size
        y_m = (row + 0.5) * self._cell_size
        return (x_m, y_m)
