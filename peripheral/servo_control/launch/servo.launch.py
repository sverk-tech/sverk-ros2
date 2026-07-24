from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from pathlib import Path


def generate_launch_description():
    config_path = Path(get_package_share_directory("servo_control")) / "config" / "servo_params.yaml"

    return LaunchDescription(
        [
            Node(
                package="servo_control",
                executable="servo_node",
                name="servo_control",
                output="screen",
                parameters=[str(config_path)],
            ),
        ]
    )
