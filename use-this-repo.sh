#!/bin/bash

sudo apt update
sudo apt upgrade -y

mkdir -p ~/ros2_ws/src

source /opt/ros/humble/setup.bash

cd ~/ros2_ws/src || exit

git clone https://github.com/elliotmiles/wheely-dan.git

cd ~/ros2_ws || exit

colcon build

grep -qxF "source /opt/ros/humble/setup.bash" ~/.bashrc || echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

grep -qxF "source ~/ros2_ws/install/setup.bash" ~/.bashrc || echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

source ~/.bashrc
