"""
ARGUS — Obstacle Detection via Floor Color Segmentation

Identifies obstacles ("debris") by comparing each frame against a calibrated
floor color profile using CIE Lab chrominance distance.  By operating in
a*b* space (ignoring L*), the detector is inherently shadow-invariant —
shadows change lightness but not chrominance.

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
    Captures a reference frame and models the floor color in CIE Lab space.
    Computes statistics of the a* and b* channels (chrominance) from a
    3×3 grid of sampled regions, ignoring L* (lightness) entirely.

    Shadow-invariant: L* is ignored during calibration and detection,
    so shadows that only change brightness don't affect results.
    """

    def __init__(self):
        # Floor chrominance model: mean and variance of a* and b*
        self._floor_a_mean = None
        self._floor_a_var = None
        self._floor_b_mean = None
        self._floor_b_var = None
        self._calibrated = False
        self._lock = threading.Lock()

    # ─────────────────────────────────────────────
    @property
    def calibrated(self):
        with self._lock:
            return self._calibrated

    @property
    def floor_ab_stats(self):
        """Return (a_mean, a_var, b_mean, b_var) or None."""
        with self._lock:
            if not self._calibrated:
                return None
            return (self._floor_a_mean, self._floor_a_var,
                    self._floor_b_mean, self._floor_b_var)

    # ─────────────────────────────────────────────
    def calibrate(self, frame, robot_bbox=None):
        """
        Snapshot the floor colour using CIE Lab chrominance.
        Samples a 3×3 grid of regions, collects all valid floor pixels,
        and computes the mean and variance of a* and b* channels.

        Args:
            frame:      BGR image (numpy array).
            robot_bbox: Optional (x1, y1, x2, y2) to mask out the robot.
        """
        blurred = cv2.GaussianBlur(frame, (15, 15), 0)
        lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2Lab)
        fh, fw = lab.shape[:2]

        # Build robot exclusion mask
        robot_mask = np.zeros((fh, fw), dtype=bool)
        if robot_bbox is not None:
            x1, y1, x2, y2 = [int(v) for v in robot_bbox]
            pad = 30
            x1 = max(0, x1 - pad)
            y1 = max(0, y1 - pad)
            x2 = min(fw, x2 + pad)
            y2 = min(fh, y2 + pad)
            robot_mask[y1:y2, x1:x2] = True

        # Sample from a 3×3 grid of regions across the frame
        region_size_x = fw // 6
        region_size_y = fh // 6
        grid_positions = []
        for gy in range(3):
            for gx in range(3):
                cx = int((gx + 0.5) * fw / 3)
                cy = int((gy + 0.5) * fh / 3)
                grid_positions.append((cx, cy))

        # Collect all valid floor pixels across all regions
        all_a_pixels = []
        all_b_pixels = []

        for cx, cy in grid_positions:
            rx1 = max(0, cx - region_size_x)
            ry1 = max(0, cy - region_size_y)
            rx2 = min(fw, cx + region_size_x)
            ry2 = min(fh, cy + region_size_y)

            region_lab = lab[ry1:ry2, rx1:rx2]
            region_robot = robot_mask[ry1:ry2, rx1:rx2]

            # Exclude robot pixels
            valid_pixels = region_lab[~region_robot]
            if len(valid_pixels) < 50:
                continue

            all_a_pixels.append(valid_pixels[:, 1].astype(np.float32))
            all_b_pixels.append(valid_pixels[:, 2].astype(np.float32))

        if not all_a_pixels:
            log.warning("Not enough valid regions for calibration")
            return False

        # Concatenate all floor pixels
        a_all = np.concatenate(all_a_pixels)
        b_all = np.concatenate(all_b_pixels)

        # Compute statistics
        a_mean = float(np.mean(a_all))
        b_mean = float(np.mean(b_all))
        a_var = max(float(np.var(a_all)), 1.0)   # floor ≥ 1 to avoid div/0
        b_var = max(float(np.var(b_all)), 1.0)

        with self._lock:
            self._floor_a_mean = a_mean
            self._floor_a_var = a_var
            self._floor_b_mean = b_mean
            self._floor_b_var = b_var
            self._calibrated = True

        log.info(
            f"Floor calibrated (Lab chrominance): "
            f"a*={a_mean:.1f}±{a_var**0.5:.1f}, b*={b_mean:.1f}±{b_var**0.5:.1f}"
        )
        return True


class ObstacleDetector:
    """
    Detects obstacles using chrominance distance in CIE Lab space.
    Computes Mahalanobis distance of each pixel's (a*, b*) from the
    calibrated floor distribution.  Pixels far from the floor color
    in chrominance are obstacles; shadows only shift L* and are ignored.
    """

    def __init__(self, calibrator, pixel_to_world_fn, grid_cell_size_m=None,
                 arena_width_m=None, arena_height_m=None, clearance_cells=None,
                 chroma_threshold=None):
        from config import Config

        self._calibrator = calibrator
        self._pixel_to_world = pixel_to_world_fn

        self._cell_size = grid_cell_size_m or Config.GRID_CELL_SIZE_M
        self._arena_w = arena_width_m or Config.ARENA_WIDTH_M
        self._arena_h = arena_height_m or Config.ARENA_HEIGHT_M
        self._clearance = clearance_cells if clearance_cells is not None else Config.ROBOT_CLEARANCE_CELLS
        self._chroma_threshold = chroma_threshold or Config.FLOOR_CHROMA_THRESHOLD

        # Grid dimensions
        self._grid_cols = int(np.ceil(self._arena_w / self._cell_size))
        self._grid_rows = int(np.ceil(self._arena_h / self._cell_size))

        # Temporal smoothing: keep last N masks, require consistent detection
        self._mask_history = deque(maxlen=5)  # last 5 frames
        self._smoothing_threshold = 0.6       # pixel must be debris in 60% of frames

        # Pre-compute morphological kernels (avoid per-frame allocation)
        self._kernel_erode = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self._kernel_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        if self._clearance > 0:
            ksize = int(2 * self._clearance + 1)
            self._inflate_kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (ksize, ksize)
            )
        else:
            self._inflate_kernel = None

        # Latest results
        self._debris_mask = None
        self._debris_contours = []      # cached contours for overlay rendering
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
        Process a frame to find debris using Lab chrominance distance.

        Args:
            frame:      BGR image.
            robot_bbox: (x1, y1, x2, y2) of the detected robot to exclude.

        Returns:
            True if detection ran (calibrator ready), False otherwise.
        """
        stats = self._calibrator.floor_ab_stats
        if stats is None:
            return False

        a_mean, a_var, b_mean, b_var = stats

        # ── Optimize: Downsample frame to reduce computational load by 75% ──
        # The floor chrominance math (Mahalanobis distance) is very heavy
        # on the CPU. Operating on a 50% scaled frame cuts latency massively.
        scale = 0.5
        small_frame = cv2.resize(frame, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)

        # Blur to smooth out noise (kernel scaled down appropriately)
        blurred = cv2.GaussianBlur(small_frame, (7, 7), 0)
        lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2Lab)

        # Extract a* and b* channels as float for distance computation
        a_ch = lab[:, :, 1].astype(np.float32)
        b_ch = lab[:, :, 2].astype(np.float32)

        # Mahalanobis distance in a*b* space (squared)
        # Measures how far each pixel's chrominance is from the floor.
        dist_sq = ((a_ch - a_mean) ** 2) / a_var + ((b_ch - b_mean) ** 2) / b_var

        # Threshold: pixels with high chrominance distance are debris
        small_mask = (dist_sq > self._chroma_threshold).astype(np.uint8) * 255

        # Upscale the mask back to the original frame resolution using nearest neighbour (fast)
        debris_mask = cv2.resize(small_mask, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)

        # Mask out the robot bounding box (robot is not debris)
        if robot_bbox is not None:
            x1, y1, x2, y2 = [int(v) for v in robot_bbox]
            pad = 10
            x1 = max(0, x1 - pad)
            y1 = max(0, y1 - pad)
            x2 = min(frame.shape[1], x2 + pad)
            y2 = min(frame.shape[0], y2 + pad)
            debris_mask[y1:y2, x1:x2] = 0

        # Morphological cleanup: moderate erosion + dilate
        debris_mask = cv2.erode(debris_mask, self._kernel_erode, iterations=2)
        debris_mask = cv2.dilate(debris_mask, self._kernel_dilate, iterations=1)

        # Remove very small contours (dust/scratches)
        contours, _ = cv2.findContours(debris_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = 400  # px²
        for cnt in contours:
            if cv2.contourArea(cnt) < min_area:
                cv2.drawContours(debris_mask, [cnt], -1, 0, cv2.FILLED)

        # ── Temporal smoothing ──
        binary = (debris_mask > 0).astype(np.float32)
        self._mask_history.append(binary)

        if len(self._mask_history) >= 2:
            stacked = np.stack(self._mask_history, axis=0)
            avg = np.mean(stacked, axis=0)
            debris_mask = (avg >= self._smoothing_threshold).astype(np.uint8) * 255

        # Build occupancy grid
        grid_resized = cv2.resize(
            debris_mask,
            (self._grid_cols, self._grid_rows),
            interpolation=cv2.INTER_AREA,
        )
        grid = (grid_resized > 76).astype(np.uint8)  # 76/255 ≈ 30% threshold

        # Inflate obstacles for robot clearance
        if self._inflate_kernel is not None:
            grid = cv2.dilate(grid, self._inflate_kernel, iterations=1)

        obstacle_count = int(np.sum(grid))

        # Cache contours for overlay rendering
        contours, _ = cv2.findContours(
            debris_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        with self._lock:
            self._debris_mask = debris_mask
            self._debris_contours = contours
            self._occupancy_grid = grid
            self._obstacle_count = obstacle_count

        return True

    # ─────────────────────────────────────────────
    def get_debris_mask(self):
        """Return latest binary debris mask (pixel-space), or None."""
        with self._lock:
            return self._debris_mask.copy() if self._debris_mask is not None else None

    def get_debris_contours(self):
        """Return cached debris contours (computed during detect()), or empty list."""
        with self._lock:
            return list(self._debris_contours)

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
