# Wheely Dan

This is my SLAM vehicle project. Yes the name is a [pun](https://www.steelydan.com/).

Featuring:
- Depth camera & LiDAR scanning for 3D mapping using rtabmap
- Jetson Orin Nano for onboard compute
- Sensor fusion
- Custom PCB design
- A simulation environment in gazebo

The goal is to autonomously map a room in 3D and I plan to experiment with exploration algorithms for efficiency.

## How to use this repo with ROS2 (for Ubuntu 22.04 ONLY)
1. [Install ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
2. Download [setup.sh](https://github.com/elliotmiles/wheely-dan/blob/main/setup.sh)
3. In a termnial, run `bash setup.sh` in the folder it downloaded
