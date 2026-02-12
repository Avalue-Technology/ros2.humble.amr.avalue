import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    avalue_robot_dir = get_package_share_directory('turn_on_avalue_robot')
    avalue_launch_dir = os.path.join(avalue_robot_dir, 'launch')
        
    avalue_nav_dir = get_package_share_directory('avalue_nav2')
    avalue_nav_launchr = os.path.join(avalue_nav_dir, 'launch')


    map_dir = os.path.join(avalue_nav_dir, 'map')
    map_file = LaunchConfiguration('map', default=os.path.join(
        map_dir, 'avalue.yaml'))

    param_dir = os.path.join(avalue_nav_dir, 'param','avalue_params')
    param_file = LaunchConfiguration('params', default=os.path.join(
        param_dir, 'param_mini_mec.yaml'))


    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params',
            default_value=param_file,
            description='Full path to param file to load'),
        Node(
            name='waypoint_cycle',
            package='nav2_waypoint_cycle',
            executable='nav2_waypoint_cycle',
        ),   
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [avalue_nav_launchr, '/bringup_launch.py']),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': param_file}.items(),
        ),

    ])

