#!/usr/bin/env python3
"""Fake Livox publisher based on a REAL bag frame (v3).

Loads ONE real CustomMsg frame from a ros2 bag, pre-generates N noisy
variants of it at startup, then republishes them in a loop at 10 Hz with
updated timestamps. IMU (gravity only + noise) is published at 200 Hz.

Result: LIO-SAM sees a *static* scene with the real MID360 scan pattern
(zero/no-return points, 4-line structure, real offset_time) and small
per-frame measurement noise -> stable stationary odometry, no divergence.

Usage:
  python3 bag_fake_publisher.py --bag /path/to/rosbag2_dir [--frame-index 0]
  python3 bag_fake_publisher.py --bag ... --mode lidar   # split processes
  python3 bag_fake_publisher.py --mode imu               # if GIL starves IMU

Pick --frame-index from a moment when the car was STATIONARY (usually the
first frames). Using frames from different poses would make the "static"
scene jump between publishes and break scan matching.

IMU units: check your bag (`ros2 topic echo /livox/imu --once` while playing).
  z ~ 1.0  -> [g]      -> ACCEL_UNIT_G = True
  z ~ 9.8  -> [m/s^2]  -> ACCEL_UNIT_G = False
"""

import argparse
import copy
import os
import random
import sys

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message

import rosbag2_py
from sensor_msgs.msg import Imu
from livox_ros_driver2.msg import CustomMsg

# ---------------- configuration ----------------
DEFAULT_BAG = "~/rosbag/260629/rosbag2_20260625_211818"
DEFAULT_FRAME_INDEX = 1750
LIDAR_TOPIC = "/livox/lidar"
IMU_TOPIC = "/livox/imu"
LIDAR_HZ = 10.0
IMU_HZ = 100.0
NOISE_STD = 0.01          # [m] per-axis gaussian noise added to each variant
VARIANT_POOL_SIZE = 8     # pre-generated noisy copies, cycled at runtime
ACCEL_UNIT_G = True
GRAVITY = 1.0 if ACCEL_UNIT_G else 9.81
# ------------------------------------------------


def load_base_frame(bag_path: str, topic: str, frame_index: int) -> CustomMsg:
    """Read the (frame_index)-th CustomMsg on `topic` from a ros2 bag."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id=""),  # auto-detect
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    count = 0
    while reader.has_next():
        (t, data, _stamp) = reader.read_next()
        if t != topic:
            continue
        if count == frame_index:
            return deserialize_message(data, CustomMsg)
        count += 1
    raise RuntimeError(
        f"topic '{topic}' has only {count} frames; frame_index {frame_index} not found"
    )


def make_noisy_variant(base: CustomMsg, std: float) -> CustomMsg:
    """Deep-copy the frame and jitter valid points. Zero (no-return) points
    and the header/timebase/offset_time structure are preserved as-is."""
    msg = copy.deepcopy(base)
    g = random.gauss
    for p in msg.points:
        if p.x == 0.0 and p.y == 0.0 and p.z == 0.0:
            continue  # keep no-return points exactly zero
        p.x += g(0.0, std)
        p.y += g(0.0, std)
        p.z += g(0.0, std)
    return msg


class BagFakePublisher(Node):
    def __init__(self, mode: str, bag_path: str, frame_index: int):
        super().__init__(f"bag_fake_publisher_{mode}")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        if mode in ("all", "lidar"):
            self.get_logger().info(f"Loading frame {frame_index} from {bag_path} ...")
            base = load_base_frame(bag_path, LIDAR_TOPIC, frame_index)
            self.get_logger().info(
                f"Base frame: {base.point_num} pts, frame_id='{base.header.frame_id}'. "
                f"Generating {VARIANT_POOL_SIZE} noisy variants (may take a few seconds)..."
            )
            self.pool = [make_noisy_variant(base, NOISE_STD) for _ in range(VARIANT_POOL_SIZE)]
            self.idx = 0
            self.lidar_pub = self.create_publisher(CustomMsg, LIDAR_TOPIC, qos)
            self.create_timer(1.0 / LIDAR_HZ, self.publish_lidar,
                              callback_group=MutuallyExclusiveCallbackGroup())
            self.get_logger().info("Variant pool ready.")

        if mode in ("all", "imu"):
            self.imu_pub = self.create_publisher(Imu, IMU_TOPIC, qos)
            self.create_timer(1.0 / IMU_HZ, self.publish_imu,
                              callback_group=MutuallyExclusiveCallbackGroup())

        self.imu_frame_id = "livox_frame"
        self.get_logger().info(
            f"mode={mode} | lidar {LIDAR_HZ}Hz, imu {IMU_HZ}Hz | "
            f"accel unit: {'g' if ACCEL_UNIT_G else 'm/s^2'}"
        )

    def publish_lidar(self):
        now = self.get_clock().now()
        # random order (not strict cycle) so consecutive frames differ more
        self.idx = (self.idx + random.randint(1, VARIANT_POOL_SIZE - 1)) % VARIANT_POOL_SIZE
        msg = self.pool[self.idx]
        msg.header.stamp = now.to_msg()
        msg.timebase = now.nanoseconds
        self.lidar_pub.publish(msg)

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame_id
        msg.orientation.w = 1.0
        msg.angular_velocity.x = random.gauss(0.0, 0.001)
        msg.angular_velocity.y = random.gauss(0.0, 0.001)
        msg.angular_velocity.z = random.gauss(0.0, 0.001)
        acc_noise = 0.002 * GRAVITY
        msg.linear_acceleration.x = random.gauss(0.0, acc_noise)
        msg.linear_acceleration.y = random.gauss(0.0, acc_noise)
        msg.linear_acceleration.z = GRAVITY + random.gauss(0.0, acc_noise)
        self.imu_pub.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", default=DEFAULT_BAG,
                        help=f"path to rosbag2 directory (default: {DEFAULT_BAG})")
    parser.add_argument("--frame-index", type=int, default=DEFAULT_FRAME_INDEX,
                        help="which lidar frame to use as base (pick a stationary moment)")
    parser.add_argument("--mode", choices=["all", "lidar", "imu"], default="all")
    args, _ = parser.parse_known_args()

    bag_path = os.path.expanduser(args.bag)
    if args.mode in ("all", "lidar") and not os.path.isdir(bag_path):
        print(f"error: bag directory not found: {bag_path}", file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    node = BagFakePublisher(args.mode, bag_path, args.frame_index)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()