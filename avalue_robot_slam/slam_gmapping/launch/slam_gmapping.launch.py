from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock",
            ),
            SetEnvironmentVariable(
                "RCUTILS_LOGGING_BUFFERED_STREAM",
                "1",
            ),
            Node(
                package="slam_gmapping",
                executable="slam_gmapping",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
