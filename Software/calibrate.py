#!/usr/bin/env python3
"""
ARGUS — Camera Calibration Utility
Run this once to:
  1. Calibrate camera intrinsics with a checkerboard
  2. Compute the pixel→world homography using 4 known arena corners

Usage:
  python3 calibrate.py --intrinsics    # Step 1: camera intrinsics
  python3 calibrate.py --homography    # Step 2: arena homography
  python3 calibrate.py --both          # Both steps
"""

import cv2
import numpy as np
import argparse
import os
import time

from config import Config

CALIBRATION_DIR = "calibration"


def ensure_dir():
    os.makedirs(CALIBRATION_DIR, exist_ok=True)


def calibrate_intrinsics():
    """
    Calibrate camera intrinsics using a 9x6 checkerboard pattern.
    Move the checkerboard in front of the camera, press SPACE to capture
    frames, and press Q when done (need at least 10 frames).
    """
    print("\n=== Camera Intrinsics Calibration ===")
    print("Hold a 9x6 checkerboard in view of the camera.")
    print("Press SPACE to capture a frame, Q to finish (need 10+ frames).\n")

    BOARD_SIZE = (9, 6)
    SQUARE_SIZE = 0.025  # 25mm squares

    objp = np.zeros((BOARD_SIZE[0] * BOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:BOARD_SIZE[0], 0:BOARD_SIZE[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    obj_points = []
    img_points = []

    cap = cv2.VideoCapture(Config.CAMERA_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, Config.CAMERA_RESOLUTION[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, Config.CAMERA_RESOLUTION[1])

    if not cap.isOpened():
        print("ERROR: Cannot open camera")
        return False

    frame_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, BOARD_SIZE, None)

        display = frame.copy()
        if found:
            cv2.drawChessboardCorners(display, BOARD_SIZE, corners, found)
            cv2.putText(display, "DETECTED — press SPACE to capture",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(display, "Looking for checkerboard...",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.putText(display, f"Captured: {frame_count}/10+",
                    (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        cv2.imshow("Calibration", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord(" ") and found:
            corners_refined = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            )
            obj_points.append(objp)
            img_points.append(corners_refined)
            frame_count += 1
            print(f"  Captured frame {frame_count}")

        elif key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

    if frame_count < 10:
        print(f"ERROR: Need at least 10 frames, got {frame_count}")
        return False

    print(f"\nComputing calibration from {frame_count} frames...")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, gray.shape[::-1], None, None
    )

    if ret:
        ensure_dir()
        np.savez(
            Config.CALIBRATION_FILE,
            camera_matrix=mtx,
            dist_coeffs=dist,
        )
        print(f"Calibration saved to {Config.CALIBRATION_FILE}")
        print(f"Reprojection error: {ret:.4f} pixels")
        return True
    else:
        print("ERROR: Calibration failed")
        return False


def calibrate_homography():
    """
    Compute pixel→world homography by clicking 4 known arena corners.
    The arena is 2x2m with corners at:
      Top-left:     (0, 0)
      Top-right:    (2, 0)
      Bottom-right: (2, 2)
      Bottom-left:  (0, 2)
    """
    print("\n=== Arena Homography Calibration ===")
    print("Click the 4 corners of the 2x2m arena in this order:")
    print("  1. Top-left     (0, 0)")
    print("  2. Top-right    (2, 0)")
    print("  3. Bottom-right (2, 2)")
    print("  4. Bottom-left  (0, 2)")
    print("Press Q to cancel.\n")

    WORLD_CORNERS = np.array([
        [0.0, 0.0],
        [Config.ARENA_WIDTH_M, 0.0],
        [Config.ARENA_WIDTH_M, Config.ARENA_HEIGHT_M],
        [0.0, Config.ARENA_HEIGHT_M],
    ], dtype=np.float64)

    pixel_corners = []

    def on_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(pixel_corners) < 4:
            pixel_corners.append((x, y))
            print(f"  Corner {len(pixel_corners)}: pixel=({x}, {y})")

    cap = cv2.VideoCapture(Config.CAMERA_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, Config.CAMERA_RESOLUTION[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, Config.CAMERA_RESOLUTION[1])

    if not cap.isOpened():
        print("ERROR: Cannot open camera")
        return False

    cv2.namedWindow("Homography")
    cv2.setMouseCallback("Homography", on_click)

    corner_labels = ["Top-Left (0,0)", "Top-Right (2,0)",
                     "Bottom-Right (2,2)", "Bottom-Left (0,2)"]

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        display = frame.copy()

        # Draw already-clicked corners
        for i, (px, py) in enumerate(pixel_corners):
            cv2.circle(display, (px, py), 8, (0, 255, 0), -1)
            cv2.putText(display, corner_labels[i], (px + 12, py - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        if len(pixel_corners) < 4:
            label = f"Click: {corner_labels[len(pixel_corners)]}"
            cv2.putText(display, label, (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        else:
            cv2.putText(display, "All corners set — press ENTER to save, Q to redo",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("Homography", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break
        if key == 13 and len(pixel_corners) == 4:  # ENTER
            break

    cap.release()
    cv2.destroyAllWindows()

    if len(pixel_corners) != 4:
        print("ERROR: Need exactly 4 corners")
        return False

    pixel_pts = np.array(pixel_corners, dtype=np.float64)
    H, status = cv2.findHomography(pixel_pts, WORLD_CORNERS)

    if H is not None:
        ensure_dir()
        np.savez(Config.HOMOGRAPHY_FILE, homography=H)
        print(f"\nHomography saved to {Config.HOMOGRAPHY_FILE}")

        # Verify: transform the corners back
        print("\nVerification:")
        for i, (px, py) in enumerate(pixel_corners):
            pt = np.array([[[px, py]]], dtype=np.float64)
            world = cv2.perspectiveTransform(pt, H)
            wx, wy = world[0][0]
            expected = WORLD_CORNERS[i]
            err = np.sqrt((wx - expected[0])**2 + (wy - expected[1])**2)
            print(f"  Corner {i+1}: ({wx:.3f}, {wy:.3f})m — "
                  f"expected ({expected[0]:.1f}, {expected[1]:.1f})m — "
                  f"error: {err:.4f}m")
        return True
    else:
        print("ERROR: Homography computation failed")
        return False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ARGUS Camera Calibration")
    parser.add_argument("--intrinsics", action="store_true", help="Calibrate camera intrinsics")
    parser.add_argument("--homography", action="store_true", help="Calibrate arena homography")
    parser.add_argument("--both", action="store_true", help="Run both calibrations")
    args = parser.parse_args()

    if args.both or args.intrinsics:
        calibrate_intrinsics()
    if args.both or args.homography:
        calibrate_homography()
    if not (args.both or args.intrinsics or args.homography):
        parser.print_help()
