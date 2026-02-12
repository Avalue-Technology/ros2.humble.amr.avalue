import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    avalue_nav_dir = get_package_share_directory("avalue_robot_nav2")
    avalue_nav_launch_dir = os.path.join(avalue_nav_dir, "launch")

    map_dir = os.path.join(avalue_nav_dir, "map")
    map_file = LaunchConfiguration(
        "map",
        default=os.path.join(map_dir, "avalue.yaml"),
    )

    # Modify the model parameter file, the options are:
    # param_mini_akm.yaml / param_mini_4wd.yaml / param_mini_diff.yaml /
    # param_mini_mec.yaml / param_mini_omni.yaml / param_mini_tank.yaml /
    # param_senior_akm.yaml / param_senior_diff.yaml / param_senior_mec_bs.yaml
    # param_senior_mec_dl.yaml / param_top_4wd_bs.yaml / param_top_4wd_dl.yaml
    # param_top_akm_dl.yaml / param_four_wheel_diff_dl.yaml / param_four_wheel_diff_bs.yaml
    param_dir = os.path.join(avalue_nav_dir, "param", "avalue_params")
    param_file = LaunchConfiguration(
        "params",
        default=os.path.join(param_dir, "param_mini_4wd.yaml"),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=map_file,
                description="Full path to map file to load",
            ),
            DeclareLaunchArgument(
                "params",
                default_value=param_file,
                description="Full path to param file to load",
            ),
            Node(
                name="waypoint_cycle",
                package="nav2_waypoint_cycle",
                executable="nav2_waypoint_cycle",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(avalue_nav_launch_dir, "bringup_launch.py")
                ),
                launch_arguments={
                    "map": map_file,
                    "use_sim_time": use_sim_time,
                    "params_file": param_file,
                }.items(),
            ),
        ]
    )
