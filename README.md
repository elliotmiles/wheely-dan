# Wheely Dan

This is my SLAM vehicle project. Yes the name is a [pun](https://www.steelydan.com/).

Featuring:
- Depth camera & LiDAR scanning for 3D mapping using rtabmap
- Jetson Orin Nano for onboard compute
- Sensor fusion
- Custom PCB design
- A simulation environment in gazebo

The goal is to autonomously map a room in 3D and I plan to experiment with exploration algorithms for efficiency.

## Demos



## Project Overview

At its core, everything is built primarily around the SLAM algorithm. 

The four sensors (wheel encoders, IMU, LiDAR, and depth camera) are all used in some way by the SLAM algorithm to create a map and then optimise the estimate of the robot's position within that map.

<img width="939" height="410" alt="image" src="https://github.com/user-attachments/assets/b45b9cd0-dc44-47c6-98e2-a4c636ddfa57" />


ROS2 is used as a framework to separate different functions and organise the data shared between those functions.

Below you can see a more detailed block diagram that shows the structure of the software for the robot:



Here is another block diagram that shows the hardware for the robot:

<img width="1236" height="838" alt="hardware-block-diagram" src="https://github.com/user-attachments/assets/3c39929e-a4bf-47e5-acd3-91e2e7971cb5" />


## How to use this repo with ROS2 (for Ubuntu 22.04 ONLY)
1. [Install ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
2. Download [setup.sh](https://github.com/elliotmiles/wheely-dan/blob/main/setup.sh)
3. In a termnial, run `bash setup.sh` in the folder it downloaded
