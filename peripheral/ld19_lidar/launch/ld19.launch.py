from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = PathJoinSubstitution(
        [FindPackageShare("ld19_lidar"), "config", "ld19.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "connection_mode",
                default_value="usb",
                description="LD19 connection mode: usb or uart.",
            ),
            DeclareLaunchArgument(
                "port_name",
                default_value="",
                description="Explicit serial port override. Empty means auto/default.",
            ),
            DeclareLaunchArgument("baudrate", default_value="230400"),
            DeclareLaunchArgument("frame_id", default_value="ld19_frame"),
            DeclareLaunchArgument("topic_name", default_value="/ld19_lidar/scan"),
            Node(
                package="ld19_lidar",
                executable="ld19_lidar_node",
                name="ld19_lidar_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "connection_mode": LaunchConfiguration("connection_mode"),
                        "port_name": LaunchConfiguration("port_name"),
                        "baudrate": ParameterValue(
                            LaunchConfiguration("baudrate"),
                            value_type=int,
                        ),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "topic_name": LaunchConfiguration("topic_name"),
                    },
                ],
            ),
        ]
    )
