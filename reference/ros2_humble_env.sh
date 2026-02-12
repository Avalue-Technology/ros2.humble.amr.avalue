# sudo chown renity_admin:renity_admin /home/renity_admin/ros2_humble_amr_avalue/ros2_humble_env.sh
# chmod 644 /home/renity_admin/ros2_humble_amr_avalue/ros2_humble_env.sh
# chmod +x  /home/renity_admin/ros2_humble_amr_avalue/ros2_humble_env.sh

#!/usr/bin/env bash
# Avoid Python/ROS Localization Errors
export LANG=C.UTF-8
export LC_ALL=C.UTF-8

# Display Environment: If GUI/RViz is not available, it can be removed
export DISPLAY=:0

# Basic PATH (sometimes non-interactive shells lack PATH)
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

# ROS2 Humble Environment
source /opt/ros/humble/setup.bash
# ROS Configuration (can be changed to your domain ID)
#export ROS_DISTRO=Humble
#export ROS_DOMAIN_ID=0
#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Intel RealSense ROS2, SLAMTEC LIDAR ROS2
source /home/renity_admin/ros2_ws/install/setup.bash

# Avalue AMR ROS2
source /home/renity_admin/ros2_humble_amr_avalue/install/setup.bash
