#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/autoRCcar/ros2/install/setup.bash
eval "$(grep '^export GSCAM' ~/.bashrc)"
sleep 3
exec ros2 run autorccar_utils process_manager_node
