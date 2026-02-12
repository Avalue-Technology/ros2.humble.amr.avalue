# Introduction
This article primarily introduces how to install Avalue ROS2 Humble AMR Nodes on the EMS-TGL Ubuntu 22.04 environment provided by Avalue Technology Inc.

# Prerequisite
## ROS2 Humble
Because of Avalue ROS2 Humble AMR Nodes depends on ROS2 Humble environment, please refer as follows GitHub Repository to complete preparing.


[ros2.humble.EMS-TGL](https://github.com/Avalue-Technology/ros2.humble.EMS-TGL)

## Intel® RealSense™ ROS2
Because of Avalue ROS2 Humble AMR Nodes may depends on Intel® RealSense™ ROS2 Node, please refer as follows GitHub Repository to complete preparing.


[ros2.humble.camera.intelrealsense](https://github.com/Avalue-Technology/ros2.humble.camera.intelrealsense)

## SLAMTEC LIDAR ROS2
Because of Avalue ROS2 Humble AMR Nodes depends on SLAMTEC LIDAR ROS2 Node, please refer as follows GitHub Repository to complete preparing.


[ros2.humble.lidar.slamtec](https://github.com/Avalue-Technology/ros2.humble.lidar.slamtec)

## udev rules (/etc/udev/rules.d)
Please refer to the file `avalue_udev.sh` in the `reference` folder to create symbolic link rules for peripheral devices.
E.g. Intel® RealSense™, SLAMTEC LIDAR, IMU...etc.

Because the Avalue ROS2 Humble AMR Nodes establish connections with peripherals via symbolic links, avoiding issues caused by plugging and unplugging devices or changing locations that could lead to connection failures!

# Install Dependency
```bash
sudo apt update
sudo apt install -y libasio-dev
sudo apt install -y libgoogle-glog-dev
sudo apt install -y libpcap-dev
sudo apt install -y libuvc-dev
sudo apt install -y nlohmann-json3-dev
sudo apt install -y ros-humble-rtcm-msgs
sudo apt install -y ros-humble-bond ros-humble-bondcpp
sudo apt install -y ros-humble-test-msgs
sudo apt install -y ros-humble-filters
sudo apt install -y ros-humble-nav2-msgs
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install -y ros-humble-image-common
sudo apt install -y ros-humble-pcl-ros
sudo apt install -y ros-humble-pcl-conversions
sudo apt install -y ros-humble-async-web-server-cpp
sudo apt install -y ros-humble-image-publisher
sudo apt install -y ros-humble-image-proc
sudo apt install -y ros-humble-image-transport
sudo apt install -y ros-humble-image-transport-plugins
sudo apt install -y ros-humble-behaviortree-cpp-v3
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-imu-filter-madgwick
sudo apt install ros-humble-robot-localization
```

# Build & Install - Avalue ROS2 Humble AMR Nodes
```bash
cd ~
mkdir ros2_humble_amr_avalue
cd ros2_humble_amr_avalue
# Clone - ros2.humble.amr.avalue Source Code
git clone https://github.com/Avalue-Technology/ros2.humble.amr.avalue.git .
# Install Dependency 
rosdep install --from-paths . --ignore-src -r -y
# Clean - build, install, log
rm -rf build/ install/ log/
# Compile - ros2.humble.amr.avalue Source Code - All ROS2 Nodes
colcon build --symlink-install
```

# Configure ROS2 Humble, Intel® RealSense™ ROS2, SLAMTEC LIDAR ROS2 Environment
```bash
# Setup ROS2 Humble Environment
source /opt/ros/humble/setup.bash
# Setup Intel® RealSense™ ROS2, SLAMTEC LIDAR ROS2 Environment
cd ~/ros2_ws
source install/setup.bash
# Setup Avalue ROS2 AMR Environment
cd ~/ros2_humble_amr_avalue
source install/setup.bash
```

# Usage
```bash
# Turn On - Intel® RealSense™ ROS2
ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true'
```

```bash
# Turn On - IMU ROS2
ros2 launch yesense_std_ros2 yesense_node.launch.py
```

```bash
# Turn On - SLAMTEC LIDAR ROS2 (A2 M12)
ros2 launch sllidar_ros2 view_sllidar_a2m12_launch.py
```

```bash
# Turn On - AMR ROS2
ros2 launch turn_on_avalue_robot turn_on_avalue_robot.launch.py
```

# Optional Installation - ROS2 WebSocket Bridge Server Node
If you would like to create ROS2 Web Application, you should consider to install this package.
It can help us to subscribe and operate ROS2 Node through WebSocket.

```bash
sudo apt update
# Install ROS2 Packages - Dependency
sudo apt install ros-humble-launch-xml
# Install ROS2 Packages - ros-humble-rosbridge-server
sudo apt install ros-humble-rosbridge-server

# Setup ROS2 humble Environment
source /opt/ros/humble/setup.bash

# Turn On - WebSocket Bridge Server ROS2
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

# Reference
## erase_ros_log.sh
Please refer to the file `erase_ros_log.sh` in the `reference` folder to erase ROS2 Log.

## realsense-info.sh
Please refer to the file `realsense-info.sh` in the `reference` folder to determine Intel® RealSense™: /dev/video0~/dev/video5 functionality.
E.g. RGB Camera, Depth Camera...etc.

## TF Tree (AMCL/SLAM - ROS 2 Navigation Framework)
```bash
ros2 run tf2_ros tf2_echo base_link laser_link
```

```
map
└── odom_combined (From EKF: IMU + odom0)
    └── base_footprint     (IMU filter or URDF)
	        └── base_link			
            └── wheels (or Left and right motors joint)

map (AMCL or slam)
└── odom_combined  (IMU dead reckoning)
    └── base_footprint	
         └── base_link
```
