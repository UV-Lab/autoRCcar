from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, Int8, Float32

from .submodules.user_geometry import *
from autorccar_interfaces.msg import NavState, Path, PathPoint


class Ros2Node(Node):
    def __init__(self):
        super().__init__("autorccar_gcs")
        self.nav_status_callback_fn = None  # External callback for GUI updates
        self.teleop_mode_callback_fn = None  # External callback for GUI updates

        self.publisher_command = self.create_publisher(Int8, "gcs/command", 10)
        self.publisher_setyaw = self.create_publisher(Float32, "setyaw_topic", 10)
        self.publisher_global_path = self.create_publisher(Path, "gcs/global_path", 10)
        self.subscription_nav = self.create_subscription(
            NavState, "nav_topic", self.nav_status_callback, 10
        )
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscription_teleop_mode = self.create_subscription(
            Bool, "hardware_control/teleop_mode", self.teleop_mode_callback, qos
        )

    def publish_command(self, command):
        msg = Int8()
        msg.data = command
        self.get_logger().info('pub-command: "%d"' % msg.data)
        self.publisher_command.publish(msg)

    def publish_set_yaw(self, yaw):
        msg = Float32()
        msg.data = yaw
        self.get_logger().info('pub-setYaw: "%lf"' % msg.data)
        self.publisher_setyaw.publish(msg)

    def publish_global_path(self, path_list):
        msg = Path()
        for point in path_list:
            msg.path_points.append(PathPoint(x=point[0], y=point[1], speed=0.0))
        self.publisher_global_path.publish(msg)

    def nav_status_callback(self, msg):
        if self.nav_status_callback_fn:
            self.nav_status_callback_fn(msg)

    def teleop_mode_callback(self, msg):
        if self.teleop_mode_callback_fn:
            self.teleop_mode_callback_fn(msg)
