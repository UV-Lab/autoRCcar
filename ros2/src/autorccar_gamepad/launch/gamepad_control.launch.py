from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('joy_vehicle_control'),
        'launch',
        'joy_config.yaml'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'autorepeat_rate': 20.0}],
    )

    joy_vehicle_control_node = Node(
        package='autorccar_gamepad',
        executable='joy_vehicle_control_node',
        name='joy_vehicle_control_node',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([
        joy_node,
        joy_vehicle_control_node,
    ])