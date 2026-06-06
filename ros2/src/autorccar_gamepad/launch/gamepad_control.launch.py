from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('autorccar_gamepad'),
        'launch',
        'gamepad_config.yaml'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'autorepeat_rate': 10.0}],
        output='screen',
    )

    gamepad_control_node = Node(
        package='autorccar_gamepad',
        executable='gamepad_control_node',
        name='gamepad_control_node',
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([joy_node, gamepad_control_node])