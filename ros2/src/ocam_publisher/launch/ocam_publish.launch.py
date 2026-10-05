import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory("ocam_publisher")
    params_file = LaunchConfiguration("params_file")
    publish_rviz = LaunchConfiguration("publish_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(share_dir, "config", "params.yaml"),
                description="Path to the ROS 2 parameters file.",
            ),
            DeclareLaunchArgument(
                "publish_rviz",
                default_value="true",
                description="Publish the 15 Hz JPEG visualization topic.",
            ),
            Node(
                package="ocam_publisher",
                executable="ocam_node",
                name="ocam_image_publisher",
                parameters=[params_file],
                output="screen",
                on_exit=Shutdown(reason="camera publisher exited"),
            ),
            Node(
                package="topic_tools",
                executable="drop",
                name="rviz_image_decimator",
                arguments=["/image_topic", "1", "2", "/image_topic_rviz_raw"],
                condition=IfCondition(publish_rviz),
                output="screen",
            ),
            Node(
                package="image_transport",
                executable="republish",
                name="rviz_image_republisher",
                arguments=["raw", "compressed"],
                remappings=[
                    ("in", "/image_topic_rviz_raw"),
                    ("out/compressed", "/image_topic_rviz/compressed"),
                ],
                parameters=[{"out.format": "jpeg", "out.jpeg_quality": 80}],
                condition=IfCondition(publish_rviz),
                output="screen",
            ),
        ]
    )
