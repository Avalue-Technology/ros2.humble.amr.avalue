#! /bin/bash

### BEGIN INIT
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash;source /home/avalue/avalue_ros2/install/setup.bash;ros2 launch avalue_multi avalue_slave.launch.py"
sleep 10

wait
exit 0

