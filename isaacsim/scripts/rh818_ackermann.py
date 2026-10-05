"""
RH818 Ackermann + 4WD kinematics (pure Python, no Isaac Sim dependency)

Converts a bicycle-model command (speed, center steering angle) into
FL/FR steering angles and per-wheel angular velocities.
Same geometry as test_drive.py, but in SI units (rad, rad/s).
"""

import math

L = 0.325       # wheelbase [m]
TF = 0.270      # front track [m]
TR = 0.260      # rear track [m]
RW = 0.055      # wheel radius [m]

MAX_INNER_STEER = math.radians(45.0)   # max steering angle of the inner front wheel [rad]


def _wheel_steer(lateral):
    """Steering angle of a front wheel whose lateral distance to the turn center is `lateral`."""
    angle = math.atan2(L, lateral)     # (0, pi)
    return angle - math.pi if angle > math.pi / 2.0 else angle


def ackermann_geometry(center_steer):
    """
    Return (radius, fl_steer, fr_steer) for a center steering angle [rad].
    radius is the rear-axle-midpoint turning radius [m], + = left turn.
    """
    if abs(center_steer) < 1.0e-9:
        return math.inf, 0.0, 0.0

    radius = L / math.tan(center_steer)
    fl = _wheel_steer(radius - TF / 2.0)
    fr = _wheel_steer(radius + TF / 2.0)
    return radius, fl, fr


def limit_center_steer(center_steer):
    """Limit the center steering angle so that the inner wheel does not exceed MAX_INNER_STEER."""
    if abs(center_steer) < 1.0e-9:
        return 0.0

    _, fl, fr = ackermann_geometry(center_steer)
    if max(abs(fl), abs(fr)) <= MAX_INNER_STEER:
        return center_steer

    sign = 1.0 if center_steer > 0.0 else -1.0
    low, high = 0.0, abs(center_steer)
    for _ in range(40):
        mid = 0.5 * (low + high)
        _, fl, fr = ackermann_geometry(sign * mid)
        if max(abs(fl), abs(fr)) > MAX_INNER_STEER:
            high = mid
        else:
            low = mid
    return sign * low


def compute(speed, center_steer):
    """
    speed        : rear-axle-midpoint speed [m/s]
    center_steer : bicycle-model steering angle [rad], + = left

    return dict with
      steer : {"fl", "fr"} steering joint angles [rad]
      wheel : {"fl", "fr", "rl", "rr"} wheel angular velocities [rad/s]
      center_steer : applied (limited) center steering angle [rad]
    """
    center_steer = limit_center_steer(center_steer)
    radius, fl, fr = ackermann_geometry(center_steer)

    if math.isinf(radius):
        omega = speed / RW
        wheel = {"fl": omega, "fr": omega, "rl": omega, "rr": omega}
    else:
        r = abs(radius)
        # Each wheel follows its own circle about the same center
        wheel = {
            "fl": speed * math.hypot(radius - TF / 2.0, L) / r / RW,
            "fr": speed * math.hypot(radius + TF / 2.0, L) / r / RW,
            "rl": speed * abs(radius - TR / 2.0) / r / RW,
            "rr": speed * abs(radius + TR / 2.0) / r / RW,
        }

    return {"steer": {"fl": fl, "fr": fr}, "wheel": wheel, "center_steer": center_steer}
