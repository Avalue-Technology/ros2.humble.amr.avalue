from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    bringup_dir = get_package_share_directory('turn_on_avalue_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    avalue_camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir,'launch', 'avalue_camera.launch.py')),
    )
    avalue_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_avalue_robot.launch.py')),
    )
    return LaunchDescription([
    avalue_camera,avalue_robot,
    Node(
        package='avalue_robot_kcf',
        executable='run_tracker_node',
        parameters=[{'targetDist_': 0.8}],
        output='screen',
    )
    
    ])


