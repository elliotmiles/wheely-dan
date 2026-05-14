# 18/04/2026 - SLAM

I have implemented SLAM on the robot using `slam_toolbox`. It was fairly straightforward, I just downloaded it and copied the relevant config file to my source code. 

I used the online asynchronous mode, meaning:
1. it is processing real-time data
2. it is always processing the latest scan (so not necessarily every scan received)

I initially tried to run `ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true params_file:=./src/wd_navigation/config/mapper_params_online_async.yaml`
which resulted in an error:
<img width="813" height="127" alt="Screenshot from 2026-04-18 19-17-05" src="https://github.com/user-attachments/assets/09ac8f3d-93fe-4db2-bbfc-328a4b88995f" />

Apparently that command is for an older version of ROS2 and now the correct command is `ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true slam_params_file:=./src/wd_navigation/config/mapper_params_online_async.yaml
` which yielded the correct results.





https://github.com/user-attachments/assets/aeaf4adf-0821-4af0-9df3-b328802047cf


# 15/04/2026 - ros2_control

ros2_control allows use of the same controllers across multiple hardware interfaces., i.e. with both gazebo and the real robot. 

I implemented optional ros2_control functionality onto the robot. This loads two controllers:
- **diff_cont**, which gets the wheel velocities to be sent to the hardware interface
- **joint_broad**, which reads the motor encoder data and publishes this to /joint_states (which is then used by robot state publisher)

<img width="848" height="593" alt="image" src="https://github.com/user-attachments/assets/dbc696bc-1ea6-4390-acab-8659f7c83de6" />


To make things organised, I created a new package called `wd_control`, and added `ros2_control.xacro` and `controllers.yaml`.

I then ran into a bug with inconsistent message types.
<img width="805" height="134" alt="Screenshot from 2026-04-16 14-28-47" src="https://github.com/user-attachments/assets/c09b1ef5-ed0b-4322-9dd4-d3d621351c63" />

For mentions of `/cmd_vel`, my source code was inconsistent across files with using `Twist` or `TwistStamped` as the message type. 

To fix it, I decided to use `TwistStamped` for all of them. Using the stamped version is also considered best practice. 

Next I plan to implement `slam_toolbox` for basic SLAM capability, then I will setup the hardware interfaces to get it working on the real robot.

# 27/03/2026 - LiDAR Module

I am using the RPLIDAR A1 for my robot. From what I've observed, it has good range but poor resolution. In addition, it can't pick up objects closer than 0.3m. 

To test the module, I routed the three motor cables (which will eventually end up on the main PCB) to my bench power supply, and the four UART cables to the adapter board that came with the lidar module. 
I then connected the Jetson via USB and used ssh to enter it via my desktop PC. My desktop is on ROS2 Jazzy, whereas the Jetson is on Humble and I was worried this may cause issues however it turned out to be fine.

<img width="1920" height="1440" alt="image" src="https://github.com/user-attachments/assets/3a78bdaa-8ffb-4c87-b619-ac9c66cdbd26" />

The RPLIDAR SDK is on slamtec's github page [here](https://github.com/Slamtec/rplidar_ros), however this is for ROS1 and is outdated. Instead I ran `sudo apt install ros-jazzy-rplidar-ros` to install the ROS2 package. 

I then ran the following command to start the lidar publishing node:
```
ros2 run rplidar_ros rplidar_composition   --ros-args   -p serial_port:=/dev/ttyUSB0   -p serial_baudrate:=115200   -p frame_id:=laser_frame   -p angle_compensate:=true   -p scan_mode:=Express
```
Below you can see an image of the result visualised in rviz2.

<img width="1848" height="1037" alt="Screenshot from 2026-03-27 17-47-09" src="https://github.com/user-attachments/assets/f667c942-8fea-4d80-9fe6-38c7454f17e3" />

The pointcloud data provides a map of the surroundings.

# 26/03/2026 - Gazebo Simulation

First I loaded the barebones URDF model into gazebo, with only `<visual>` tags, to check the .stl files were loading correctly.  
<img width="619" height="633" alt="Screenshot from 2026-03-18 21-56-00" src="https://github.com/user-attachments/assets/a9f6deda-b31c-48db-968c-e41ed4d23088" />

Next I added the `<collision>` and `<inertial>` tags ([see the URDF blog post](https://github.com/elliotmiles/wheely-dan/blob/main/blog/4.%20URDF.md)), and added the `gz::sim::systems::DiffDrive` plugin, which allows manual teleoperation. To begin with I used `teleop_twist_keyboard` to publish to `/cmd_vel`. 

I immediately ran into an issue: the robot was able to move forwards and backwards, but not able to turn. From this image you can see `angular { z: 1 }` meaning `/cmd_vel` was correct, so the robot was trying to turn. 
<img width="1748" height="934" alt="Screenshot from 2026-03-24 11-39-42" src="https://github.com/user-attachments/assets/52d2353c-f70e-4974-834f-63534406409d" />
I first thought it was to do with traction, then I thought it was to do with inertia. 
Turns out it was because having cylinders as the collision geometry for wheels messes with gazebo, and my caster wheels' collision geometry were cylinders. So I changed the caster wheels' collision geometry to spheres, which fixed the bug.

Once I had the robot driving, I then tried adding the lidar sensor. However I had another bug - the /scan topic was not appearing in the "Visualize lidar" section in gazebo. 
<img width="1831" height="1036" alt="Screenshot from 2026-03-25 18-35-11" src="https://github.com/user-attachments/assets/3e561111-cd42-4032-a76f-7d7c84c383b3" />
Turns out this was because I hadn't correctly included the `gz::sim::systems::Sensors` plugin. 

Once I'd fixed that, I could visualise the lidar scanner in gazebo. This publishes to the `/scan` topic, which is of type `/sensor_msgs/msg/LaserScan`. 
<img width="1850" height="1046" alt="image" src="https://github.com/user-attachments/assets/bb983810-5075-45c8-8a83-0ca39c754dee" />

I then made a quick custom world using some basic primitives to show rviz2 displaying `/scan` and `/robot_description` in `odom`.


https://github.com/user-attachments/assets/ce577a56-6b6a-4da5-9a92-93284e13d900


# 23/03/2026 - Jetson Setup

This was the process I used to setup the Jetson Orin Nano:

## 1. Flash JetPack onto the NVMe SSD. 
> https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit

## 2. Install firefox (optional)
```
sudo snap remove firefox
sudo add-apt-repository
ppa:mozillateam/ppa
sudo apt update
sudo apt install firefox
```
## 3. Fix a known snap bug (only required with step 2). 
In hindsight I probably should have avoided snap altogether.
> https://forums.developer.nvidia.com/t/neither-chromium-nor-firefox-work-with-my-jetson-orin-nano/338669

## 4. Install ROS2 Humble. 
> https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

## 5. Add automatic ROS2 sourcing.
Append `source /opt/ros/humble/setup.bash` to `~.bashrc`

## 6. Install VScode, selecting .deb Arm64.
> https://code.visualstudio.com/Download

## 7. SSH into the Jetson from my PC for headless operation and telemetry broadcast.
```
ssh <username>@<ip>
```

## 8. Clone my github repos

`git clone https://github.com/<username>/<repo-name>.git`

# 21/03/2026 - URDF

For the simulation, I need to create a URDF file that describes the robot. This will then publish a topic called /robot_description to ROS2. 

I split the CAD file up into parts that would be appropriate to have its own link, for example the wheel must be a separate link from the chassis because it rotates. 
I had trouble combining the motor drivers into one body in CAD since the file was made up of surfaces (not solids), so I just exported them as .stl files and made them separate links to save time.

Each link is described by three tags: visual, collision, and inertial. 

The visual and collision tags are straightforward because I can use the .stl file for each link. The inertial tag requires moment of inertia about each axis (that is, $Ixx$, $Iyy$, and $Izz$), mass, and the location of the centre of mass relative to the local link origin. 
Most links can be modelled as either a box or a cylinder. 

Through integration, $I = \int r^2 \, dm$, which leads to the standard results for basic shapes:

For a box of dimensions (x, y, z), 

$$
I_{xx} = \frac{1}{12} m (y^2 + z^2)
$$

$$
I_{yy} = \frac{1}{12} m (x^2 + z^2)
$$

$$
I_{zz} = \frac{1}{12} m (x^2 + y^2)
$$

For a solid cylinder of height $h$ and radius $r$ (where the long axis is in the y direction),

$$
I_{yy} = \frac{1}{2} m r^2
$$

$$
I_{xx} = I_{zz} = \frac{1}{12} m (3r^2 + h^2)
$$

For a solid cylinder of height $h$ and radius $r$ (where the long axis is in the z direction),

$$
I_{zz} = \frac{1}{2} m r^2
$$

$$
I_{xx} = I_{yy} = \frac{1}{12} m (3r^2 + h^2)
$$

> Notes:
> - URDF uses SI units (kg, m, kg m²).
> - The convention is x forwards, y left, and z up.

Here is a table for the links simplified to a box:

| Link | Mass / kg | Dimensions "x y z" / m | Ixx / kg m<sup>2</sup> | Iyy / kg m<sup>2</sup> | Izz / kg m<sup>2</sup> | COM coords "x y z" / m |  
| ---- | --------- | ---------------------- | ---------------------- | ---------------------- | ---------------------- | ---------------------- |
| jetson_link | 0.182 | 0.0905 0.103 0.0348 | 1.79E-04 | 1.43E-04 | 2.85E-04 | 0 0 0.005 |
| battery_link | 0.178 | 0.034 0.105 0.023 | 1.71E-04 | 2.50E-05 | 1.81E-04 | 0 0 0.0115 |
| pcb_link | 0.04 | 0.08 0.085 0.0016 | 2.41E-05 | 2.13E-05 | 4.54E-05 | 0 0 0 |
| driver_left_link | 0.02 | 0.033 0.061 0.005 | 6.24E-06 | 1.86E-06 | 8.02E-06 | 0 0 0 |
| driver_right_link | 0.02 | 0.033 0.061 0.005 | 6.24E-06 | 1.86E-06 | 8.02E-06 | 0 0 0 |
| lidar_link | 0.187 | 0.1 0.07 0.055 | 1.23E-04 | 2.03E-04 | 2.32E-04 | 0 0 0.0275 | 

Here is a table for the links simplified to a cylinder:

| Link | Mass / kg | Radius / m | Height / m | Ixx / kg m<sup>2</sup> | Iyy / kg m<sup>2</sup> | Izz / kg m<sup>2</sup> | COM coords "x y z" / m |  
| ---- | --------- | ---------- | ---------- | ---------------------- | ---------------------- | ---------------------- | ---------------------- |
| wheel_left_link | 0.046 | 0.0325 | 0.025 | 1.45E-05 | 2.43E-05 | 1.45E-05 | 0 0.003 0 |
| wheel_right_link | 0.046 | 0.0325 | 0.025 | 1.45E-05 | 2.43E-05 | 1.45E-05 | 0 -0.003 0 |
| caster_fork_left_link | 0.01 | 0.0095 | 0.008 | 2.79E-07 | 2.79E-07 | 4.51E-07 | 0 0 -0.004 |
| caster_wheel_left_link | 0.004 | 0.006 | 0.009 | 6.30E-08 | 7.20E-08 | 6.30E-08 | 0 0 0 |
| caster_fork_right_link | 0.01 | 0.0095 | 0.008 | 2.79E-07 | 2.79E-07 | 4.51E-07 | 0 0 -0.004 |
| caster_wheel_right_link | 0.004 | 0.006 | 0.009 | 6.30E-08 | 7.20E-08 | 6.30E-08 | 0 0 0 |

The chassis is too complex a shape to be simplified to a box, so I took the inertial properties directly out of CAD (it may not be perfect because I had to select one material for the entire part when in reality it has multiple).

| Link | Mass / kg | Ixx / kg m<sup>2</sup> | Iyy / kg m<sup>2</sup> | Izz / kg m<sup>2</sup> | COM coords "x y z" / m |  
| ---- | --------- | ---------------------- | ---------------------- | ---------------------- | ---------------------- |
| chassis_link | 1.227 | 3.86E-03 | 6.37E-03 | 8.57E-03 | 0.097 0.003 0.029 |

# 16/03/2026 - CAD

I designed the robot in Fusion 360, using a mix of metal parts and 3D printed parts. Three aluminium layers form the layout of the robot, while 3D printed mounts hold the components. This is for two reasons: 

1. I do not want to screw breakout boards directly to aluminium in case of a short circuit.
2. 3D printing allows much faster prototyping and I can re-print parts that may have been miscalculated with a fast turnaround.

The robot is differential drive, meaning it is driven by the two back wheels and has two casters at the front.
The bottom layer is reserved for power electronics and any high-current cables, the middle layer holds the main PCB board and the computing power, while the top layer holds sensors (so far only a LiDAR, but I can also add depth cameras in the future). 
This design has multiple benefits:
  1. The IMU is close to the axis of rotation (the back wheels). This means it can easily distinguish between linear acceleration and angular velocity.
  2. The IMU is far from EMI that may be caused by the power electronics.
  3. The LiDAR has an unobstructed view.
  4. The battery and motors are kept low, meaning the centre of mass of the robot is relatively low, reducing its tendency to topple.

I also added a fan on the bottom layer for extra cooling, since high current can cause heating. 

<img width="1041" height="732" alt="design" src="https://github.com/user-attachments/assets/c3e2bd23-528e-43db-8d54-1741f8347b69" />
<img width="1920" height="1080" alt="render1" src="https://github.com/user-attachments/assets/9583541d-e8e8-4825-b6da-de9093015cf8" />
<img width="1920" height="1080" alt="render2" src="https://github.com/user-attachments/assets/43138677-e76d-405c-826b-14036573b97e" />
<img width="1920" height="1080" alt="render3" src="https://github.com/user-attachments/assets/05909c98-569e-4c87-887c-3407384478c5" />

The next step will be to export parts as .stl files and then write a URDF file that joins them together and assigns properties such as mass. This will allow me to simulate the robot in Gazebo.  

# 13/03/2026 - PCB Design

## Schematic
After realising grove headers would be unncessary for the LiDAR and motor encoder connections, I switched to using screw terminals for those connections instead. I edited the schematic so that now some terminals require through-hole pads for screw terminals, with the rest requiring SMD pads for grove headers. 

Here is the updated schematic:
<img width="1117" height="851" alt="image" src="https://github.com/user-attachments/assets/dac2bb4a-b067-4731-a63d-7f89b9292e92" />


## PCB layout
I opted to go with a two-layer PCB simply because the board is not very large and so two layers is sufficient. 

<img width="891" height="735" alt="image" src="https://github.com/user-attachments/assets/c8032f1b-8b7a-46d0-a2ac-e43aef461e11" />

I then found the appropriate .step files online and assigned those to each footprint to ensure there were no collisions. I also wanted to keep the design relatively compact.

<img width="910" height="770" alt="image" src="https://github.com/user-attachments/assets/d6fb3106-20f9-49b7-beca-1ef0abab50d2" />

I decided to get 5 identical PCBs just in case anything went wrong. The PCBs arrived a couple of weeks after ordering from JLCPCB, and I immediately got to soldering. Below you can see a bare PCB next to a fully assembled one.

<img width="1920" height="1440" alt="image" src="https://github.com/user-attachments/assets/21dbeb39-ddf5-4daf-8628-a4edc9eb5de3" />

# 05/03/2026 - Schematic Design

I used KiCad for the schematic and PCB design simply because it is free and widely used. The PCB acts as a hub for the robot, despite only containing the MCU and IMU onboard. All off-board connections will be centered around grove headers. This means that all "logic" (low current) connections will be grove wires, allowing easy distinction between high-current and low-current carrying wires.

## Schematic

I found the symbol for the Teensy online, and made the symbol for the IMU myself. I found symbols for the grove connectors online, and the symbol representing the screw terminal for the board's 5V power rail is a generic connector. 

Instead of wiring all GND pins together, I connected all GND pins to the dedicated GND symbol. This is so that I can create a "ground pour", where all the empty space on a layer is connected to GND, which is ideal because it means that all GND connections share a low impedance. 

<img width="1182" height="862" alt="image" src="https://github.com/user-attachments/assets/95bf9756-5157-46d5-bf1e-5ccb3175c909" />

