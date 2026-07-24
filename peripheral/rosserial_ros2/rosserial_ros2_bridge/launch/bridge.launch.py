from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    reconnect_delay = LaunchConfiguration("reconnect_delay")
    max_payload_size = LaunchConfiguration("max_payload_size")
    topic_request_interval = LaunchConfiguration("topic_request_interval")
    stats_log_interval = LaunchConfiguration("stats_log_interval")
    diagnostics_topic = LaunchConfiguration("diagnostics_topic")
    diagnostics_period = LaunchConfiguration("diagnostics_period")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyS4"),
        DeclareLaunchArgument("baud", default_value="57600"),
        DeclareLaunchArgument("reconnect_delay", default_value="2.0"),
        DeclareLaunchArgument("max_payload_size", default_value="4096"),
        DeclareLaunchArgument("topic_request_interval", default_value="1.0"),
        DeclareLaunchArgument("stats_log_interval", default_value="10.0"),
        DeclareLaunchArgument("diagnostics_topic", default_value="/rosserial_ros2_bridge/diagnostics"),
        DeclareLaunchArgument("diagnostics_period", default_value="1.0"),
        Node(
            package="rosserial_ros2_bridge",
            executable="bridge_node",
            name="rosserial_bridge",
            output="screen",
            parameters=[
                {
                    "port": port,
                    "baud": ParameterValue(baud, value_type=int),
                    "reconnect_delay": ParameterValue(reconnect_delay, value_type=float),
                    "max_payload_size": ParameterValue(max_payload_size, value_type=int),
                    "topic_request_interval": ParameterValue(topic_request_interval, value_type=float),
                    "stats_log_interval": ParameterValue(stats_log_interval, value_type=float),
                    "diagnostics_topic": diagnostics_topic,
                    "diagnostics_period": ParameterValue(diagnostics_period, value_type=float),
                }
            ],
        ),
    ])
