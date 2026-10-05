"""
RH818 IMU / GNSS simulation (replaces the u-blox F9R board and autorccar_ubloxf9r)

Publishes the same topics and messages as autorccar_ubloxf9r
  IMU   autorccar_interfaces/Imu    100 Hz
        angular_velocity [rad/s], linear_acceleration [m/s^2] (specific force, -9.8 on z at rest)
        + white sensor noise + speed-dependent driving vibration (fitted to the real-car rosbag)
        IMU board frame: forward = x, right = y, down = z (FRD)
  GNSS  autorccar_interfaces/Gnss   5 Hz
        position_ecef [m], velocity_ecef [m/s] (WGS84 ECEF)
  timestamp = simulation time (autorccar_ins_gnss only uses the message timestamps)

Sensor positions come from the asset frames (build_car_usd.py)
  IMU   base_link/sensors/imu_link            (ZED-F9R internal IMU)
  GNSS  base_link/sensors/gnss_antenna_link   (lever arm applied to position / velocity)

Sim world frame is ENU at ORIGIN_LLH: world X = east, Y = north, Z = up.
The vehicle spawns facing +X, i.e. heading east.

Must be imported after SimulationApp is created.
"""

import math

import numpy as np


# =============================================================================
# Parameters
# =============================================================================

IMU_HZ = 100.0
GNSS_HZ = 5.0

# Sensor frames in the vehicle asset (relative to the vehicle root)
IMU_FRAME = "base_link/sensors/imu_link"
GNSS_FRAME = "base_link/sensors/gnss_antenna_link"
BASE_FRAME = "base_link"

# Sim world origin (lat [deg], lon [deg], height [m]).
# First GNSS fix of ~/Downloads/rosbag2_2026_06_02/rosbag2_1970_01_01-09_10_53
ORIGIN_LLH = (37.38204506, 127.15083373, 97.02)

# IMU noise = white sensor noise + speed-dependent driving vibration (all values 1 sigma, per axis x, y, z)
#   white  : sensor noise, measured while the vehicle was stationary in the same rosbag
#   vib_a, vib_b : driving vibration (road / motor / suspension), std = vib_a + vib_b * speed,
#                  faded to 0 below IMU_VIB_FADE_SPEED. Set both to 0 to disable vibration.
#                  Fitted to straight-driving segments of the rosbag (1 s high-passed IMU vs GNSS speed 0.4-2.0 m/s),
#                  e.g. real std at 1.27 m/s: acc (0.44, 0.42, 1.10) m/s^2, gyro (0.131, 0.111, 0.044) rad/s
#   vib_ar : vibration is first-order AR noise with this lag-1 autocorrelation at 100 Hz (most power in 2-35 Hz)
ACC_NOISE = {
    "white": 0.0035,                 # [m/s^2]
    "vib_a": (0.60, 0.28, 0.34),     # [m/s^2]
    "vib_b": (0.0, 0.13, 0.57),      # [m/s^2 per m/s]
    "vib_ar": (0.4, 0.6, 0.5),
}
GYRO_NOISE = {
    "white": 0.00037,                # [rad/s]
    "vib_a": (0.033, 0.039, 0.018),  # [rad/s]
    "vib_b": (0.075, 0.061, 0.015),  # [rad/s per m/s]
    "vib_ar": (0.7, 0.7, 0.8),
}
IMU_VIB_FADE_SPEED = 0.2  # [m/s]

GNSS_POS_NOISE = 0.0    # [m]  (rosbag position was constant while stationary)
GNSS_VEL_NOISE = 0.011  # [m/s]

# The real driver stores ECEF values as 32-bit float (0.25 to 0.5 m steps for position)
GNSS_FLOAT32 = True

# IMU board orientation in imu_link (wxyz): 180 deg about x, FLU -> FRD
IMU_ORIENTATION = (0.0, 1.0, 0.0, 0.0)


# =============================================================================
# Geodesy (WGS84)
# =============================================================================

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def llh_to_ecef(lat_deg, lon_deg, h):
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(lat) ** 2)
    return np.array([
        (n + h) * math.cos(lat) * math.cos(lon),
        (n + h) * math.cos(lat) * math.sin(lon),
        (n * (1.0 - WGS84_E2) + h) * math.sin(lat),
    ])


def enu_to_ecef_rotation(lat_deg, lon_deg):
    """Rotation matrix whose columns are the east, north, up unit vectors in ECEF."""
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    sl, cl = math.sin(lat), math.cos(lat)
    so, co = math.sin(lon), math.cos(lon)
    return np.array([
        [-so, -sl * co, cl * co],
        [co, -sl * so, cl * so],
        [0.0, cl, sl],
    ])


def quat_to_matrix(q):
    """Rotation matrix from a wxyz quaternion."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


# =============================================================================
# Sensor publisher
# =============================================================================

def create_imu_prim(car_path):
    """Author the IMU sensor prim under imu_link (call before the timeline starts)."""
    from isaacsim.sensors.experimental.physics import IMU

    path = f"{car_path}/{IMU_FRAME}/imu_sensor"
    IMU.create(path, translations=[[0.0, 0.0, 0.0]], orientations=[IMU_ORIENTATION])
    return path


def get_gnss_offset(stage, car_path):
    """GNSS antenna position in base_link [m], read from the asset."""
    from pxr import UsdGeom

    antenna = stage.GetPrimAtPath(f"{car_path}/{GNSS_FRAME}")
    base = stage.GetPrimAtPath(f"{car_path}/{BASE_FRAME}")
    if not antenna.IsValid() or not base.IsValid():
        raise RuntimeError(f"GNSS antenna frame not found: {car_path}/{GNSS_FRAME}")
    tf, _ = UsdGeom.XformCache().ComputeRelativeTransform(antenna, base)
    return np.array(tf.ExtractTranslation())


class SensorPublisher:
    """Samples the IMU / vehicle state on physics steps and publishes IMU and GNSS messages."""

    def __init__(self, node, car, imu_path, gnss_offset, physics_hz, seed=None):
        from autorccar_interfaces.msg import Gnss, Imu
        from isaacsim.core.simulation_manager import IsaacEvents, SimulationManager
        from isaacsim.sensors.experimental.physics import IMUSensor

        self.imu_every = self._steps_per_sample(physics_hz, IMU_HZ)
        self.gnss_every = self._steps_per_sample(physics_hz, GNSS_HZ)

        self.Imu, self.Gnss = Imu, Gnss
        self.sim_manager = SimulationManager
        self.car = car
        self.imu_sensor = IMUSensor(imu_path)
        self.imu_pub = node.create_publisher(Imu, "IMU", 10)
        self.gnss_pub = node.create_publisher(Gnss, "GNSS", 10)
        self.rng = np.random.default_rng(seed)
        self.acc_vib_state = np.zeros(3)     # unit-variance AR(1) states
        self.gyro_vib_state = np.zeros(3)

        self.origin_ecef = llh_to_ecef(*ORIGIN_LLH)
        self.r_enu_ecef = enu_to_ecef_rotation(ORIGIN_LLH[0], ORIGIN_LLH[1])
        self.gnss_offset = np.asarray(gnss_offset, dtype=float)
        self.step = 0

        self.callback_id = SimulationManager.register_callback(self._on_physics_step, IsaacEvents.POST_PHYSICS_STEP)
        print(
            f"[RH818] Sensors       : IMU {physics_hz / self.imu_every:.0f} Hz, "
            f"GNSS {physics_hz / self.gnss_every:.0f} Hz, origin {ORIGIN_LLH}"
        )
        print(f"[RH818] IMU prim      : {imu_path}")
        print(f"[RH818] GNSS offset   : {np.round(self.gnss_offset, 4)} m in base_link")

    @staticmethod
    def _steps_per_sample(physics_hz, rate_hz):
        steps = physics_hz / rate_hz
        if abs(steps - round(steps)) > 1.0e-6:
            raise ValueError(f"--physics-hz {physics_hz:g} must be a multiple of the sensor rate {rate_hz:g} Hz")
        return int(round(steps))

    def shutdown(self):
        self.sim_manager.deregister_callback(self.callback_id)

    def _on_physics_step(self, dt, context):
        self.step += 1
        if self.step % self.imu_every == 0:
            self._publish_imu()
        if self.step % self.gnss_every == 0:
            self._publish_gnss()

    def _stamp(self, msg):
        t = self.sim_manager.get_simulation_time()
        msg.timestamp.sec = int(t)
        msg.timestamp.nanosec = int(round((t - int(t)) * 1.0e9)) % 1000000000

    def _publish_imu(self):
        r = self.imu_sensor.get_sensor_reading()
        if not r.is_valid:
            return

        acc = np.array([r.linear_acceleration_x, r.linear_acceleration_y, r.linear_acceleration_z])
        gyro = np.array([r.angular_velocity_x, r.angular_velocity_y, r.angular_velocity_z])
        lin_vel, _ = self.car.get_velocities()
        speed = float(np.linalg.norm(lin_vel.numpy()[0][:2]))
        acc += self._noise(ACC_NOISE, self.acc_vib_state, speed)
        gyro += self._noise(GYRO_NOISE, self.gyro_vib_state, speed)

        msg = self.Imu()
        self._stamp(msg)
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = map(float, gyro)
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = map(float, acc)
        self.imu_pub.publish(msg)

    def _noise(self, cfg, vib_state, speed):
        """White sensor noise + speed-dependent colored vibration. vib_state is updated in place."""
        phi = np.asarray(cfg["vib_ar"])
        # AR(1) with unit stationary variance: x_k = phi * x_(k-1) + sqrt(1 - phi^2) * w_k
        vib_state[:] = phi * vib_state + np.sqrt(1.0 - phi**2) * self.rng.standard_normal(3)

        fade = min(speed / IMU_VIB_FADE_SPEED, 1.0)
        vib_std = fade * (np.asarray(cfg["vib_a"]) + np.asarray(cfg["vib_b"]) * speed)
        return self.rng.normal(0.0, cfg["white"], 3) + vib_std * vib_state

    def _publish_gnss(self):
        positions, orientations = self.car.get_world_poses()
        lin_vel, ang_vel = self.car.get_velocities()
        p = positions.numpy()[0].astype(float)
        rot = quat_to_matrix(orientations.numpy()[0].astype(float))
        v = lin_vel.numpy()[0].astype(float)
        w = ang_vel.numpy()[0].astype(float)

        # Antenna position / velocity in the ENU world frame (lever arm from base_link)
        lever = rot @ self.gnss_offset
        p_enu = p + lever + self.rng.normal(0.0, GNSS_POS_NOISE, 3)
        v_enu = v + np.cross(w, lever) + self.rng.normal(0.0, GNSS_VEL_NOISE, 3)

        p_ecef = self.origin_ecef + self.r_enu_ecef @ p_enu
        v_ecef = self.r_enu_ecef @ v_enu
        if GNSS_FLOAT32:
            p_ecef = p_ecef.astype(np.float32)
            v_ecef = v_ecef.astype(np.float32)

        msg = self.Gnss()
        self._stamp(msg)
        msg.position_ecef.x, msg.position_ecef.y, msg.position_ecef.z = map(float, p_ecef)
        msg.velocity_ecef.x, msg.velocity_ecef.y, msg.velocity_ecef.z = map(float, v_ecef)
        self.gnss_pub.publish(msg)
