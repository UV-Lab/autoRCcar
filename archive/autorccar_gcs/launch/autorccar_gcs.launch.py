"""GCS launcher for Auto RC Car."""

import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    gcs_node = Node(
        package="autorccar_gcs",
        executable="autorccar_gcs",
        name="autorccar_gcs",
        additional_env={"PYTHONUNBUFFERED": "1"},
        output="screen",
    )
    ld = LaunchDescription()
    ld.add_action(gcs_node)

    return ld
