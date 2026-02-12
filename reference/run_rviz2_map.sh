#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /home/renity_admin/ros2_humble_amr_avalue/install/setup.bash

export QT_QPA_PLATFORM=xcb


CONFIG_FILE="/home/renity_admin/ros2_humble_amr_avalue/avalue_ros2_humble.rviz"
exec rviz2 -d "$CONFIG_FILE"
