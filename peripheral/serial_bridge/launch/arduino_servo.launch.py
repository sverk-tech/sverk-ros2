from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    package_share = Path(get_package_share_directory("serial_bridge"))
    params_file = package_share / "config" / "arduino_servo.yaml"

    return LaunchDescription(
        [
            Node(
                package="serial_bridge",
                executable="arduino_servo_node",
                name="serial_bridge",
                output="screen",
                parameters=[str(params_file)],
            )
        ]
    )
