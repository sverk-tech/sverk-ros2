from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port_name",
                default_value="",
                description="UART port override. Empty tries /dev/ttyAMA3, /dev/ttyAMA0, /dev/serial0, /dev/ttyAMA10, /dev/ttyS0.",
            ),
            DeclareLaunchArgument("baudrate", default_value="230400"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("ld19_lidar"), "launch", "ld19.launch.py"]
                    )
                ),
                launch_arguments={
                    "connection_mode": "uart",
                    "port_name": LaunchConfiguration("port_name"),
                    "baudrate": LaunchConfiguration("baudrate"),
                }.items(),
            ),
        ]
    )
