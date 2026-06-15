#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/autoRCcar/ros2/install/setup.bash
exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml
