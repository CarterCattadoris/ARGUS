"""
ARGUS — PID Autopilot Controller
Dual PID loops: steering (heading correction) + throttle (distance-based speed).
Outputs differential drive motor commands for the 2WD chassis.
Signed floats: positive = forward, negative = reverse, per motor.
"""

import math
import logging

log = logging.getLogger("argus.pid")


class PID:
    """Generic PID controller with anti-windup and output clamping."""

    def __init__(self, kp, ki, kd, output_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit

        self._integral = 0.0
        self._prev_error = None
        self._integral_limit = output_limit * 2  # anti-windup cap

    def compute(self, error, dt):
        """
        Compute PID output given current error and timestep.
        Returns clamped output in [-output_limit, +output_limit].
        """
        if dt <= 0:
            return 0.0

        # Proportional
        p = self.kp * error

        # Integral with anti-windup
        self._integral += error * dt
        self._integral = max(-self._integral_limit,
                             min(self._integral_limit, self._integral))
        i = self.ki * self._integral

        # Derivative (on error, with zero-init)
        if self._prev_error is not None:
            d = self.kd * (error - self._prev_error) / dt
        else:
            d = 0.0
        self._prev_error = error

        output = p + i + d
        return max(-self.output_limit, min(self.output_limit, output))

    def reset(self):
        self._integral = 0.0
        self._prev_error = None


class PIDAutopilot:
    """
    Autopilot that converts (position, heading, target) into
    differential drive motor commands {left, right}.

    Steering PID: corrects heading error (angle to target vs current heading).
    Throttle PID: controls approach speed based on distance to target.

    Motor values are signed floats in [-1.0, 1.0]:
        positive = forward, negative = reverse.
    """

    def __init__(self, kp=1.2, ki=0.05, kd=0.3,
                 throttle_kp=0.8, throttle_ki=0.02, throttle_kd=0.1):
        from config import Config

        self.steering_pid = PID(kp, ki, kd, output_limit=Config.PID_OUTPUT_LIMIT)
        self.throttle_pid = PID(throttle_kp, throttle_ki, throttle_kd, output_limit=1.0)

        self.base_throttle = Config.BASE_THROTTLE
        self.min_throttle = Config.MIN_THROTTLE
        self.max_throttle = Config.MAX_THROTTLE
        self.approach_slow_dist = Config.APPROACH_SLOW_DIST
        self.docking_threshold = Config.DOCKING_THRESHOLD
        self.heading_tolerance = Config.HEADING_TOLERANCE_DEG

        self.last_steering_error = 0.0
        self.last_throttle_error = 0.0
        self._docked = False

    def compute(self, position, heading, target, dt):
        """
        Compute motor commands to navigate from position to target.

        Args:
            position: (x_m, y_m) current world position
            heading: current heading in degrees (0=north, CW)
            target: (x_m, y_m) target world position
            dt: timestep in seconds

        Returns:
            dict: {"left": -1.0 to 1.0, "right": -1.0 to 1.0,
                   "steering_correction": float, "distance": float}
        """
        dx = target[0] - position[0]
        dy = target[1] - position[1]
        distance = math.sqrt(dx * dx + dy * dy)

        # Check if docked
        if distance < self.docking_threshold:
            if not self._docked:
                log.info(f"DOCKED — distance: {distance:.3f}m")
                self._docked = True
            return {"left": 0.0, "right": 0.0,
                    "steering_correction": 0.0, "distance": distance}

        self._docked = False

        # ── Desired heading (angle from position to target) ──
        # atan2(dx, -dy) gives angle from north (up), clockwise
        desired_heading = math.degrees(math.atan2(dx, -dy)) % 360

        # ── Heading error (shortest angular distance) ──
        heading_error = desired_heading - heading
        # Normalize to [-180, 180]
        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360

        self.last_steering_error = heading_error

        # ── Decide forward vs reverse ──
        # If target is behind us (>120° error), reverse is more efficient
        reverse = abs(heading_error) > 120
        if reverse:
            # Flip the error for reverse driving
            effective_error = heading_error - 180 if heading_error > 0 else heading_error + 180
        else:
            effective_error = heading_error

        # ── Steering PID ──
        steering = self.steering_pid.compute(effective_error / 180.0, dt)

        # ── Throttle ──
        # Distance-based throttle with approach slowdown
        self.last_throttle_error = distance
        if distance < self.approach_slow_dist:
            # Linear ramp-down as we approach
            throttle_scale = distance / self.approach_slow_dist
            throttle = self.min_throttle + (self.base_throttle - self.min_throttle) * throttle_scale
        else:
            throttle = self.base_throttle

        # Reduce throttle when heading error is large (turn in place first)
        heading_factor = max(0.1, 1.0 - abs(effective_error) / 90.0)
        throttle *= heading_factor

        # Clamp throttle
        throttle = max(self.min_throttle, min(self.max_throttle, throttle))

        # ── Differential drive mixing ──
        # steering > 0 → turn right → left faster, right slower
        # steering < 0 → turn left → left slower, right faster
        left_power = throttle + steering * 0.5
        right_power = throttle - steering * 0.5

        # If heading error is very large, do a sharper turn (pivot)
        if abs(effective_error) > 45:
            turn_boost = min(1.0, abs(effective_error) / 90.0)
            if effective_error > 0:  # need to turn right
                left_power = throttle * turn_boost
                right_power = -throttle * turn_boost * 0.5
            else:  # need to turn left
                left_power = -throttle * turn_boost * 0.5
                right_power = throttle * turn_boost

        # ── Apply reverse: negate both motors ──
        if reverse:
            left_power = -left_power
            right_power = -right_power

        # ── Clamp to [-1, 1] — sign preserved ──
        left_power = max(-1.0, min(1.0, left_power))
        right_power = max(-1.0, min(1.0, right_power))

        return {
            "left": round(left_power, 3),
            "right": round(right_power, 3),
            "steering_correction": round(steering, 3),
            "distance": round(distance, 3),
        }

    def reset(self):
        """Reset PID state (call when disengaging autopilot)."""
        self.steering_pid.reset()
        self.throttle_pid.reset()
        self.last_steering_error = 0.0
        self.last_throttle_error = 0.0
        self._docked = False
        log.info("Autopilot PID reset")