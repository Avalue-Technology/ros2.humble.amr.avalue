#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source /home/renity_admin/ros2_humble_amr_avalue/install/setup.bash

exec ros2 launch avalue_robot_nav2 save_map.launch.py