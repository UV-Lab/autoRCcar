#!/bin/bash
### Dependencies

## LIO-SAM (ros2)
sudo apt install -y ros-humble-perception-pcl \
  	   ros-humble-pcl-msgs \
  	   ros-humble-vision-opencv \
  	   ros-humble-xacro

## LIO-SAM (gtsam)
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt install -y libgtsam-dev libgtsam-unstable-dev

## oCam
sudo apt-get install libv4l-dev libudev-dev

## GCS
pip install numpy PyQt5 pyqtgraph

## CostMap
sudo apt-get install -y libeigen3-dev libyaml-cpp-dev \
		ros-humble-vision-msgs

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
