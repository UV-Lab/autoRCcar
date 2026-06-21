"""Teleop Keyboard launcher for Auto RC Car."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():

    hardware_control_config_file_path = os.path.join(
        get_package_share_directory("autorccar_hardware_control"),
        "launch",
        "hardware_control.yaml",
    )
    autorccar_keyboard_cmd = [
        "gnome-terminal",
        "--",
        "ros2",
        "run",
        "autorccar_keyboard",
        "keyboard_control",
        "--ros-args",
        "--params-file",
        hardware_control_config_file_path,
    ]
    teleop_keyboard_process = ExecuteProcess(
        cmd=autorccar_keyboard_cmd, output="screen"
    )
    ld = LaunchDescription()
    ld.add_action(teleop_keyboard_process)

    return ld
