# -*- coding: utf-8 -*-

"""
RH818 Autonomous RC Car - Isaac Sim 6.1.0
Drive test: straight -> left turn -> straight -> right turn -> straight -> stop

Target vehicle asset
  autoRCcar_RH818.usda

Latest measured parameters
  Wheelbase    : 0.325 m
  Front track  : 0.270 m
  Rear track   : 0.260 m
  Wheel radius : 0.055 m
  Steering     : front Ackermann, max +/-45 deg

Usage
1) Open an Isaac Sim scene that contains a PhysicsScene and a collision ground.
2) Add autoRCcar_RH818.usda to the scene as a Reference/Payload.
3) Run this script from Window > Script Editor.
4) Press Play if the timeline is not already running.

Drive sequence
  1 s contact settling
  1.5 s soft start (0 -> 1.0 m/s)
  3 s straight
  4 s left turn (+20 deg)
  3 s straight
  4 s right turn (-20 deg)
  3 s straight
  stop and pause the timeline

Notes
- This script does not use timeline.get_current_time().
- It uses the asyncio monotonic wall clock,
  but phase time does not advance while the timeline is paused.
- USD drive targets are written directly.
"""

import asyncio
import math

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import UsdPhysics


# =============================================================================
# Vehicle parameters
# =============================================================================

L = 0.325       # wheelbase [m]
TF = 0.270      # front track [m]
TR = 0.260      # rear track [m]
RW = 0.055      # wheel radius [m]

MAX_INNER_DEG = 45.0

# Test command
TEST_SPEED_MPS = 1.0
TEST_STEER_DEG = 20.0

# Direction correction
# Set WHEEL_SIGN = -1.0 if the vehicle moves backward.
# Set STEER_SIGN = -1.0 if a positive steering command turns the vehicle right.
WHEEL_SIGN = 1.0
STEER_SIGN = 1.0

# Phase duration [seconds]
DUR_STRAIGHT_1 = 3.0
DUR_LEFT       = 4.0
DUR_STRAIGHT_2 = 3.0
DUR_RIGHT      = 4.0
DUR_STRAIGHT_3 = 3.0

# Startup transient suppression
# Allow contact constraints to settle immediately after Play starts,
# then ramp smoothly from 0 to TEST_SPEED_MPS to reduce the initial jump.
SETTLE_TIME_SEC = 1.0
START_RAMP_TIME_SEC = 1.5
RAMP_STEPS = 45

AUTO_PAUSE_AT_END = True


# =============================================================================
# Stage / vehicle discovery
# =============================================================================

stage = omni.usd.get_context().get_stage()

if stage is None:
    raise RuntimeError("No USD stage is open.")


def find_vehicle_root():
    """
    Automatically find the vehicle root that contains */joints/front_left_steer_joint.
    Works whether the asset is opened directly or referenced into another scene.
    """
    for prim in stage.Traverse():
        if (
            prim.GetName() == "front_left_steer_joint"
            and prim.GetParent().GetName() == "joints"
        ):
            return str(prim.GetParent().GetParent().GetPath())

    raise RuntimeError(
        "Could not find the RH818 vehicle joints.\n"
        "Make sure autoRCcar_RH818.usda is included in the current stage."
    )


CAR = find_vehicle_root()
JOINTS = f"{CAR}/joints"

print(f"[RH818] Vehicle root: {CAR}")


# =============================================================================
# Scene validation
# =============================================================================

def has_physics_scene():
    return any(prim.IsA(UsdPhysics.Scene) for prim in stage.Traverse())


if not has_physics_scene():
    print(
        "[RH818] WARNING: No PhysicsScene was found in the current stage.\n"
        "        Add a PhysicsScene and collision ground before running the test."
    )


# =============================================================================
# USD drive helper
# =============================================================================

def _get_drive_attr(joint_name, attr_name):
    path = f"{JOINTS}/{joint_name}"
    prim = stage.GetPrimAtPath(path)

    if not prim.IsValid():
        raise RuntimeError(f"Joint not found: {path}")

    attr = prim.GetAttribute(f"drive:angular:physics:{attr_name}")

    if not attr.IsValid():
        raise RuntimeError(
            f"Drive attribute not found:\n"
            f"  {path}\n"
            f"  drive:angular:physics:{attr_name}"
        )

    return attr


def _set(joint_name, attr_name, value):
    _get_drive_attr(joint_name, attr_name).Set(float(value))


# =============================================================================
# Ackermann + 4WD controller
# =============================================================================

def normalize_steer_deg(angle_deg):
    """
    Normalize the steering angle to the [-90, +90] branch.
    """
    angle_deg = float(angle_deg)

    while angle_deg > 90.0:
        angle_deg -= 180.0

    while angle_deg < -90.0:
        angle_deg += 180.0

    return angle_deg


def ackermann_geometry(center_steer_deg):
    """
    Convert the virtual center steering angle (bicycle model) into
    FL/FR steering angles and the rear-axle-midpoint turning radius.

    return:
        radius, fl_deg, fr_deg

    radius:
        + = left turn
        - = right turn
    """
    center_steer_deg = float(center_steer_deg)

    if abs(center_steer_deg) < 1.0e-9:
        return math.inf, 0.0, 0.0

    radius = L / math.tan(math.radians(center_steer_deg))

    fl_deg = math.degrees(
        math.atan2(L, radius - TF / 2.0)
    )
    fr_deg = math.degrees(
        math.atan2(L, radius + TF / 2.0)
    )

    fl_deg = normalize_steer_deg(fl_deg)
    fr_deg = normalize_steer_deg(fr_deg)

    return radius, fl_deg, fr_deg


def limit_center_steer(center_steer_deg):
    """
    Limit the center steering angle so that the Ackermann inner-wheel angle
    does not exceed MAX_INNER_DEG.
    """
    requested = float(center_steer_deg)

    if abs(requested) < 1.0e-9:
        return 0.0

    _, fl, fr = ackermann_geometry(requested)

    if max(abs(fl), abs(fr)) <= MAX_INNER_DEG:
        return requested

    # Binary search while preserving the sign of the requested steering angle
    # so the inner-wheel angle stays near MAX_INNER_DEG.
    sign = 1.0 if requested > 0.0 else -1.0
    low = 0.0
    high = abs(requested)

    for _ in range(40):
        mid = 0.5 * (low + high)
        _, fl_mid, fr_mid = ackermann_geometry(sign * mid)

        if max(abs(fl_mid), abs(fr_mid)) > MAX_INNER_DEG:
            high = mid
        else:
            low = mid

    return sign * low


def drive(v_mps, steer_deg=0.0):
    """
    Low-level vehicle drive interface.

    Parameters
    ----------
    v_mps : float
        Target speed at the rear-axle midpoint [m/s]

    steer_deg : float
        Virtual center steering angle from the bicycle model [deg]
        + = left turn
        - = right turn

    Operation
    ----
    1) Convert center steering angle to FL/FR Ackermann steering angles
    2) Compute each wheel trajectory radius
    3) Compute individual FL/FR/RL/RR wheel angular velocities
    4) Write USD angular-drive targetPosition / targetVelocity

    Returns
    -------
    dict
        Information about the command actually applied
    """

    user_v = float(v_mps)
    user_steer = float(steer_deg)

    # Apply direction-convention correction
    v_cmd = user_v * WHEEL_SIGN
    steer_cmd = user_steer * STEER_SIGN

    # Steering limit
    limited_steer = limit_center_steer(steer_cmd)

    # -------------------------------------------------------------------------
    # Straight
    # -------------------------------------------------------------------------
    if abs(limited_steer) < 1.0e-9:

        radius = math.inf
        fl_deg = 0.0
        fr_deg = 0.0

        omega = v_cmd / RW

        wheel_omega = {
            "fl": omega,
            "fr": omega,
            "rl": omega,
            "rr": omega,
        }

    # -------------------------------------------------------------------------
    # Turn
    # -------------------------------------------------------------------------
    else:

        radius, fl_deg, fr_deg = ackermann_geometry(limited_steer)
        abs_radius = abs(radius)

        # rear wheel trajectory radii
        r_rl = abs(radius - TR / 2.0)
        r_rr = abs(radius + TR / 2.0)

        # front wheel trajectory radii
        r_fl = math.hypot(radius - TF / 2.0, L)
        r_fr = math.hypot(radius + TF / 2.0, L)

        # Approximate differential action using individual wheel target velocities
        wheel_omega = {
            "fl": v_cmd * r_fl / abs_radius / RW,
            "fr": v_cmd * r_fr / abs_radius / RW,
            "rl": v_cmd * r_rl / abs_radius / RW,
            "rr": v_cmd * r_rr / abs_radius / RW,
        }

    # -------------------------------------------------------------------------
    # Steering targets
    # -------------------------------------------------------------------------

    _set(
        "front_left_steer_joint",
        "targetPosition",
        fl_deg
    )

    _set(
        "front_right_steer_joint",
        "targetPosition",
        fr_deg
    )

    # -------------------------------------------------------------------------
    # Wheel velocity targets
    # USD angular drive targetVelocity unit = deg/s
    # -------------------------------------------------------------------------

    wheel_joint_map = {
        "fl": "front_left_wheel_joint",
        "fr": "front_right_wheel_joint",
        "rl": "rear_left_wheel_joint",
        "rr": "rear_right_wheel_joint",
    }

    wheel_deg_s = {}

    for key, joint_name in wheel_joint_map.items():

        deg_s = math.degrees(wheel_omega[key])

        _set(
            joint_name,
            "targetVelocity",
            deg_s
        )

        wheel_deg_s[key] = deg_s

    return {
        "speed_mps": user_v,
        "requested_steer_deg": user_steer,
        "applied_center_steer_deg": (
            limited_steer / STEER_SIGN
            if abs(STEER_SIGN) > 1.0e-12
            else limited_steer
        ),
        "fl_steer_deg": fl_deg,
        "fr_steer_deg": fr_deg,
        "radius_m": radius,
        "wheel_deg_s": wheel_deg_s,
    }


def stop():
    return drive(0.0, 0.0)


# =============================================================================
# Timing
# =============================================================================

async def wait_play_seconds(seconds):
    """
    Wait for the requested amount of time accumulated only while the timeline is playing.

    Behavior:
    - Does not depend on timeline.get_current_time().
    - Pausing the timeline also pauses the phase timer.
    - Resuming Play continues from the remaining phase time.
    """
    seconds = float(seconds)

    timeline = omni.timeline.get_timeline_interface()
    app = omni.kit.app.get_app()
    loop = asyncio.get_running_loop()

    elapsed = 0.0
    last_wall = loop.time()

    while elapsed < seconds:

        await app.next_update_async()

        now = loop.time()

        if timeline.is_playing():
            elapsed += now - last_wall

        last_wall = now


# =============================================================================
# Test phase helper
# =============================================================================

async def command_phase(name, duration, speed, steer):

    result = drive(speed, steer)

    radius_text = (
        "inf"
        if math.isinf(result["radius_m"])
        else f'{result["radius_m"]:+.3f}'
    )

    print("")
    print(
        f"[RH818] {name:<11} | "
        f"duration={duration:.1f}s | "
        f"v={speed:+.2f} m/s | "
        f"center steer={steer:+.1f} deg"
    )

    print(
        f"         Ackermann  | "
        f"FL={result['fl_steer_deg']:+.2f} deg, "
        f"FR={result['fr_steer_deg']:+.2f} deg, "
        f"R={radius_text} m"
    )

    w = result["wheel_deg_s"]

    print(
        f"         wheel deg/s| "
        f"FL={w['fl']:+.1f}, "
        f"FR={w['fr']:+.1f}, "
        f"RL={w['rl']:+.1f}, "
        f"RR={w['rr']:+.1f}"
    )

    await wait_play_seconds(duration)


# =============================================================================
# Startup settling / soft start
# =============================================================================

async def settle_vehicle():
    """
    After Play starts, keep wheel targets at zero to let contacts settle.
    SETTLE_TIME_SEC is accumulated only while the timeline is playing.
    """
    stop()
    print(f"[RH818] SETTLE      | {SETTLE_TIME_SEC:.1f}s, wheel target = 0")
    await wait_play_seconds(SETTLE_TIME_SEC)


async def soft_start(target_speed, duration, steps):
    """
    Ramp wheel target speed linearly from 0 to target_speed while steering is centered.
    This reduces contact/drive impulses caused by a sudden targetVelocity step.
    """
    target_speed = float(target_speed)
    duration = float(duration)
    steps = max(1, int(steps))

    print(
        f"[RH818] SOFT START  | "
        f"0.00 -> {target_speed:.2f} m/s in {duration:.1f}s"
    )

    dt = duration / steps

    for i in range(1, steps + 1):
        speed = target_speed * i / steps
        drive(speed, 0.0)
        await wait_play_seconds(dt)

# =============================================================================
# Main drive sequence
# =============================================================================

async def run_test():

    timeline = omni.timeline.get_timeline_interface()
    app = omni.kit.app.get_app()

    # Write safe zero targets immediately after the script starts
    stop()

    print("")
    print("=" * 78)
    print("RH818 DRIVE SEQUENCE TEST")
    print(f"  Vehicle root : {CAR}")
    print(f"  Speed        : {TEST_SPEED_MPS:.2f} m/s")
    print(f"  Steering     : +/- {TEST_STEER_DEG:.1f} deg")
    print(f"  Wheelbase    : {L:.3f} m")
    print(f"  Front track  : {TF:.3f} m")
    print(f"  Rear track   : {TR:.3f} m")
    print(f"  Wheel radius : {RW:.3f} m")
    print("  Timing       : asyncio wall clock, counted only while Timeline is Play")
    print("=" * 78)

    if not timeline.is_playing():
        print("[RH818] Press Play.")

    # Wait for Play
    while not timeline.is_playing():
        await app.next_update_async()

    # PhysX initialization
    for _ in range(5):
        await app.next_update_async()

    try:

        # Important: do not command 1 m/s immediately after Play starts.
        # Settle contacts first, then ramp speed smoothly before starting the normal sequence.
        await settle_vehicle()
        await soft_start(
            TEST_SPEED_MPS,
            START_RAMP_TIME_SEC,
            RAMP_STEPS,
        )

        await command_phase(
            "STRAIGHT 1",
            DUR_STRAIGHT_1,
            TEST_SPEED_MPS,
            0.0,
        )

        await command_phase(
            "LEFT TURN",
            DUR_LEFT,
            TEST_SPEED_MPS,
            +TEST_STEER_DEG,
        )

        await command_phase(
            "STRAIGHT 2",
            DUR_STRAIGHT_2,
            TEST_SPEED_MPS,
            0.0,
        )

        await command_phase(
            "RIGHT TURN",
            DUR_RIGHT,
            TEST_SPEED_MPS,
            -TEST_STEER_DEG,
        )

        await command_phase(
            "STRAIGHT 3",
            DUR_STRAIGHT_3,
            TEST_SPEED_MPS,
            0.0,
        )

        stop()

        print("")
        print("[RH818] STOP")

        # Allow PhysX one frame to process the zero target
        await app.next_update_async()

        if AUTO_PAUSE_AT_END:
            timeline.pause()
            print("[RH818] Test complete -> Timeline paused")
        else:
            print("[RH818] Test complete -> vehicle stopped")

    except asyncio.CancelledError:

        stop()
        print("[RH818] Previous test task cancelled -> vehicle stopped")
        raise

    except Exception as exc:

        stop()
        print(f"[RH818] ERROR: {exc}")
        raise


# =============================================================================
# Script Editor re-run safety
# =============================================================================

try:
    previous_task = globals().get("_rh818_drive_test_task")

    if (
        previous_task is not None
        and not previous_task.done()
    ):
        previous_task.cancel()

except Exception:
    pass


_rh818_drive_test_task = asyncio.ensure_future(
    run_test()
)
