#!/bin/bash

## ROS2 Packages
if [ -x "$(command -v ./ros2/src/livox_ros_driver2/build.sh)" ]; then
    ./ros2/src/livox_ros_driver2/build.sh humble
else
    echo "[livox_ros_driver2/build.sh] not found"
    exit 1
fi

cd ros2
colcon build --packages-select livox_ros_driver2
colcon build --symlink-install --packages-skip livox_ros_driver2
source install/setup.bash
