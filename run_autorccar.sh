#!/bin/bash

echo "Select the navigation to launch:"
echo "1) GNSS/INS"
echo "2) LIO_SAM"
read -p "Enter choice [1 or 2]: " input


if [ "$input" == "1" ]; then
	echo "Starting ublox Driver..."
	gnome-terminal -- bash -c "ros2 run autorccar_ubloxf9r ubloxf9r; exec bash"
	sleep 2

	echo "Starting GNSS/INS Navigation..."
	gnome-terminal -- bash -c "ros2 launch autorccar_ins_gnss ins_gnss_nav.launch.py; exec bash"
	sleep 2

elif [ "$input" == "2" ]; then
	# 1. Livox MID-360
	echo "Starting Livox Driver..."
	gnome-terminal -- bash -c "ros2 launch livox_ros_driver2 msg_MID360_launch.py; exec bash"
	sleep 2

	# 2. LIO-SAM
	echo "Starting LIO-SAM..."
	gnome-terminal -- bash -c "ros2 launch lio_sam run.launch.py; exec bash"
	sleep 2
else
    echo "Invalid selection. Exiting."
fi

# 3. Costmap
echo "Starting Costmap..."
gnome-terminal -- bash -c "ros2 launch autorccar_costmap costmap.launch.py; exec bash"
sleep 1

# 4. Path Planning
echo "Starting Path-Planning..."
gnome-terminal -- bash -c "ros2 launch autorccar_planning_control planning_control.launch.py; exec bash"
sleep 1

# 5. Hardware Control
echo "Starting Hardware Control..."
gnome-terminal -- bash -c "ros2 launch autorccar_hardware_control hardware_control.launch.py; exec bash"

echo "All nodes have been launched in separate terminals."
