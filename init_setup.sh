#!/bin/bash
### Dependencies (Ubuntu 24.04 / ROS 2 Jazzy)
ROS_DISTRO=${ROS_DISTRO:-jazzy}
cd "$(dirname "$0")"

## LIO-SAM (ros2)
sudo apt install -y ros-${ROS_DISTRO}-perception-pcl \
  	   ros-${ROS_DISTRO}-pcl-msgs \
  	   ros-${ROS_DISTRO}-vision-opencv \
  	   ros-${ROS_DISTRO}-xacro

## LIO-SAM (gtsam)
# borglab PPA has no noble release; use the ROS-packaged GTSAM 4.2 instead
sudo apt install -y ros-${ROS_DISTRO}-gtsam

## e-con camera
sudo apt install -y v4l-utils
sudo apt install -y ros-${ROS_DISTRO}-gscam

## oCam
sudo apt-get install -y libv4l-dev libudev-dev

## GCS
# Ubuntu 24.04 blocks system-wide pip installs (PEP 668); use apt packages
sudo apt install -y python3-numpy python3-pyqt5 python3-pyqtgraph

## Gamepad (DualSens)
sudo apt install -y joystick
sudo apt install -y ros-${ROS_DISTRO}-joy

## CostMap
sudo apt-get install -y libeigen3-dev libyaml-cpp-dev \
		ros-${ROS_DISTRO}-vision-msgs

## Rosbridge
sudo apt-get install -y ros-${ROS_DISTRO}-rosbridge-suite

## Livox-SDK2
cd Livox-SDK2
if [ -d "build" ]; then
    rm -rf build
fi
mkdir build && cd build
cmake .. && make -j
sudo make install
cd ../..

## e-consystem
GSCAM_LINE='export GSCAM_CONFIG="v4l2src device=/dev/video0 io-mode=2 ! video/x-raw,width=1280,height=720,framerate=60/1,format=UYVY ! videoconvert"'

# 중복 체크 후 추가 (grep으로 확인)
if ! grep -qF "$GSCAM_LINE" ~/.bashrc; then
    echo "Adding GSCAM_CONFIG to .bashrc..."
    echo "$GSCAM_LINE" >> ~/.bashrc
    # 현재 터미널 세션에도 바로 적용
    eval "$GSCAM_LINE"
else
    echo "GSCAM_CONFIG already exists in .bashrc."
fi

##./build_ros2.sh
