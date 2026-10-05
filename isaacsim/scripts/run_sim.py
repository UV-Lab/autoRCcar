"""
RH818 Autonomous RC Car - Isaac Sim 6.1.0 standalone launcher
Builds a scene with a flat ground plane and the RH818 vehicle asset, then runs the simulation.
The vehicle is driven by the commands that autorccar_hardware_control publishes over ROS 2.

Scene
  /World/physicsScene   PhysicsScene (gravity -Z 9.81 m/s^2)
  /World/GroundPlane    flat collision ground plane
  /World/DomeLight, /World/DistantLight
  /World/rc_car         Reference to isaacsim/assets/autoRCcar_RH818.usda

ROS 2 interface (replaces the serial link to the vehicle MCU)
  sub hardware_control/control_command  autorccar_interfaces/ControlCommand
                                        speed [m/s], steering_angle [rad] after hardware_control clamping
  sub gcs/command, teleop/command       std_msgs/Int8, stop commands zero the vehicle
                                        (same as the MCU receiving a stop drive command)
  pub IMU                               autorccar_interfaces/Imu, 100 Hz, FRD body frame
  pub GNSS                              autorccar_interfaces/Gnss, 5 Hz, WGS84 ECEF
                                        (replaces autorccar_ubloxf9r, see rh818_sensors.py)

Usage (from the repository root, ROS 2 workspace must be sourced for autorccar_interfaces)
  source ros2/install/setup.bash
  ~/isaacsim/python.sh isaacsim/scripts/run_sim.py                    # GUI, runs until the window is closed
  ~/isaacsim/python.sh isaacsim/scripts/run_sim.py --headless --duration 10

  # GCS manual control in another terminal
  ros2 launch autorccar_hardware_control hardware_control.launch.py   # use_dummy_hardware: true in hardware_control.yaml
  ./start_rosbridge.sh
  ./GCS/GCS_Linux_Release/autorccar_gcs

Options
  --headless          Run without a window
  --duration SEC      Simulation time to run [s]. 0 = until the window is closed (GUI only).
                      Default: 0 in GUI mode, 3 in headless mode
  --physics-hz HZ     Physics steps per second, multiple of 100 (default 200)
  --usd PATH          Vehicle asset path (default ../assets/autoRCcar_RH818.usda)
  --no-play           GUI only: build the scene but do not start the timeline
  --no-ros2           Do not use ROS 2 (no drive commands, no IMU/GNSS topics)

Notes
- The vehicle asset has no ground or PhysicsScene; this script provides both.
- The asset places base_link at axle height (z = 0.055 m), so the vehicle root is spawned at z = 0
  plus a small drop height to avoid initial penetration.
- Like the real ESC/servo, the last received command is held until a new one arrives.
- The sim world frame is ENU (X = east, Y = north, Z = up); the vehicle starts heading east.
"""

import argparse
import math
import os
import sys
import time

parser = argparse.ArgumentParser(description="Run RH818 on a flat ground plane in Isaac Sim")
parser.add_argument("--headless", action="store_true", help="run without a window")
parser.add_argument("--duration", type=float, default=None, help="simulation time to run [s], 0 = forever")
parser.add_argument("--physics-hz", type=float, default=200.0, help="physics steps per second")
parser.add_argument(
    "--usd",
    default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assets", "autoRCcar_RH818.usda"),
    help="vehicle asset path",
)
parser.add_argument("--no-play", action="store_true", help="GUI only: do not start the timeline")
parser.add_argument("--no-ros2", action="store_true", help="do not use ROS 2")
args, _ = parser.parse_known_args()

if args.duration is None:
    args.duration = 3.0 if args.headless else 0.0
if args.headless and args.duration <= 0.0:
    parser.error("--duration must be > 0 in headless mode")

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.headless})

# Omniverse imports must come after SimulationApp is created
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.timeline
from isaacsim.core.experimental.objects import DistantLight, DomeLight, GroundPlane
from isaacsim.core.experimental.prims import Articulation
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rh818_ackermann as ackermann
import rh818_sensors as sensors


# =============================================================================
# Parameters
# =============================================================================

CAR_PATH = "/World/rc_car"
SPAWN_POS = [0.0, 0.0, 0.005]   # small drop height above the ground [m]
GRAVITY = 9.81                  # [m/s^2]

STEER_JOINTS = {"fl": "front_left_steer_joint", "fr": "front_right_steer_joint"}
WHEEL_JOINTS = {
    "fl": "front_left_wheel_joint",
    "fr": "front_right_wheel_joint",
    "rl": "rear_left_wheel_joint",
    "rr": "rear_right_wheel_joint",
}


# =============================================================================
# Scene construction
# =============================================================================

def build_scene(usd_path):
    usd_path = os.path.abspath(usd_path)
    if not os.path.isfile(usd_path):
        raise FileNotFoundError(f"Vehicle asset not found: {usd_path}")

    stage_utils.create_new_stage()
    stage = stage_utils.get_current_stage()

    # Match the asset convention: Z-up, meters, kilograms
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)

    UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

    # Physics scene
    scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(GRAVITY)
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx_scene.CreateTimeStepsPerSecondAttr(int(args.physics_hz))

    # Flat ground and lights
    GroundPlane("/World/GroundPlane", positions=[0.0, 0.0, 0.0])
    DomeLight("/World/DomeLight").set_intensities(800)
    DistantLight("/World/DistantLight").set_intensities(2500)

    # Vehicle
    stage_utils.add_reference_to_stage(usd_path=usd_path, path=CAR_PATH)
    UsdGeom.XformCommonAPI(stage.GetPrimAtPath(CAR_PATH)).SetTranslate(Gf.Vec3d(*SPAWN_POS))

    # IMU board
    app_utils.enable_extension("isaacsim.sensors.experimental.physics")
    imu_path = sensors.create_imu_prim(CAR_PATH)

    print(f"[RH818] Vehicle asset : {usd_path}")
    print(f"[RH818] Physics       : {args.physics_hz:.0f} Hz")
    return imu_path


def wait_stage_loading():
    while stage_utils.is_stage_loading():
        simulation_app.update()


def print_pose(car, t):
    positions, _ = car.get_world_poses()
    p = positions.numpy()[0]
    print(f"[RH818] t={t:6.2f}s  base pos = ({p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f}) m")


# =============================================================================
# Vehicle drive
# =============================================================================

class VehicleDriver:
    """Writes Ackermann steering / 4WD wheel velocity targets to the articulation."""

    def __init__(self, car):
        self.car = car
        self.steer_idx = [int(car.get_dof_indices(STEER_JOINTS[k]).numpy()[0]) for k in ("fl", "fr")]
        self.wheel_idx = [int(car.get_dof_indices(WHEEL_JOINTS[k]).numpy()[0]) for k in ("fl", "fr", "rl", "rr")]
        self.speed = 0.0
        self.steer = 0.0

    def set_command(self, speed, steer):
        self.speed = float(speed)
        self.steer = float(steer)

    def apply(self):
        """Write the current command to the joint drives (call every frame while playing)."""
        cmd = ackermann.compute(self.speed, self.steer)
        self.car.set_dof_position_targets(
            [[cmd["steer"]["fl"], cmd["steer"]["fr"]]], dof_indices=self.steer_idx
        )
        self.car.set_dof_velocity_targets(
            [[cmd["wheel"][k] for k in ("fl", "fr", "rl", "rr")]], dof_indices=self.wheel_idx
        )


# =============================================================================
# ROS 2 command interface
# =============================================================================

class Ros2CommandInterface:
    """Subscribes to hardware_control output and stop commands, and forwards them to the driver."""

    def __init__(self, driver):
        app_utils.enable_extension("isaacsim.ros2.bridge")
        simulation_app.update()

        import rclpy
        from std_msgs.msg import Int8

        try:
            from autorccar_interfaces.msg import ControlCommand
        except ImportError as exc:
            raise RuntimeError(
                "autorccar_interfaces not found. Run 'source ros2/install/setup.bash' "
                "before launching this script, or pass --no-ros2."
            ) from exc

        self.rclpy = rclpy
        self.driver = driver

        rclpy.init()
        self.node = rclpy.create_node("rh818_sim_drive")
        self.node.create_subscription(
            ControlCommand, "hardware_control/control_command", self._on_control_command, 10
        )
        self.node.create_subscription(Int8, "gcs/command", lambda m: self._on_drive_command("gcs", m), 10)
        self.node.create_subscription(Int8, "teleop/command", lambda m: self._on_drive_command("teleop", m), 10)
        print("[RH818] ROS 2         : subscribed to hardware_control/control_command, gcs/command, teleop/command")

    def _on_control_command(self, msg):
        if msg.speed != self.driver.speed or msg.steering_angle != self.driver.steer:
            print(
                f"[RH818] command       : speed={msg.speed:+.2f} m/s, "
                f"steer={math.degrees(msg.steering_angle):+.1f} deg"
            )
        self.driver.set_command(msg.speed, msg.steering_angle)

    def _on_drive_command(self, source, msg):
        # Only "1" (start) keeps the vehicle running; every other value makes hardware_control stop
        if msg.data != 1:
            print(f"[RH818] {source} command {msg.data} -> stop")
            self.driver.set_command(0.0, 0.0)

    def spin(self):
        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def shutdown(self):
        self.node.destroy_node()
        self.rclpy.try_shutdown()


class RealTimePacer:
    """Sleeps so that simulation time does not run ahead of wall-clock time."""

    MAX_LAG = 0.1   # re-anchor when sim falls behind by more than this [s] (e.g. after a pause)

    def __init__(self):
        self.anchor = None

    def wait(self, sim_time):
        now = time.monotonic()
        if self.anchor is None or sim_time < self.anchor[1]:
            self.anchor = (now, sim_time)
            return

        ahead = (sim_time - self.anchor[1]) - (now - self.anchor[0])
        if ahead > 0.0:
            time.sleep(ahead)
        elif ahead < -self.MAX_LAG:
            self.anchor = (now, sim_time)


# =============================================================================
# Main
# =============================================================================

ros2 = None
sensor_pub = None

try:
    imu_path = build_scene(args.usd)
    wait_stage_loading()

    car = Articulation(CAR_PATH)
    print(f"[RH818] DOF names     : {car.dof_names}")

    if args.headless or not args.no_play:
        app_utils.play()
        simulation_app.update()     # allow physics to initialize

    driver = VehicleDriver(car)
    if not args.no_ros2:
        ros2 = Ros2CommandInterface(driver)
        gnss_offset = sensors.get_gnss_offset(stage_utils.get_current_stage(), CAR_PATH)
        sensor_pub = sensors.SensorPublisher(ros2.node, car, imu_path, gnss_offset, args.physics_hz)

    timeline = omni.timeline.get_timeline_interface()
    pacer = RealTimePacer()
    next_print = 0.0

    while simulation_app.is_running():
        if ros2 is not None:
            ros2.spin()

        if app_utils.is_playing():
            driver.apply()

        simulation_app.update()

        if not app_utils.is_playing():
            continue

        sim_time = timeline.get_current_time()
        pacer.wait(sim_time)

        if args.headless and sim_time >= next_print:
            print_pose(car, sim_time)
            next_print += 0.5

        if args.duration > 0.0 and sim_time >= args.duration:
            print_pose(car, sim_time)
            break

finally:
    if sensor_pub is not None:
        sensor_pub.shutdown()
    if ros2 is not None:
        ros2.shutdown()
    simulation_app.close()
