#!/usr/bin/env python3
import ast

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_literal(value, expected_name):
    try:
        return ast.literal_eval(value)
    except (ValueError, SyntaxError) as exc:
        raise RuntimeError(f'Failed to parse launch argument {expected_name}={value!r}') from exc


def _build_nodes(context, *args, **kwargs):
    xshut_pins = list(_parse_literal(LaunchConfiguration('xshut_pins').perform(context), 'xshut_pins'))
    input_topics = list(_parse_literal(LaunchConfiguration('input_topics').perform(context), 'input_topics'))
    sensor_angles_deg = list(
        _parse_literal(LaunchConfiguration('sensor_angles_deg').perform(context), 'sensor_angles_deg')
    )
    sensor_fov_deg = list(
        _parse_literal(LaunchConfiguration('sensor_fov_deg').perform(context), 'sensor_fov_deg')
    )

    projection_mode = LaunchConfiguration('projection_mode').perform(context)
    overlap_mode = LaunchConfiguration('overlap_mode').perform(context)
    output_topic = LaunchConfiguration('output_topic').perform(context)
    frame_id = LaunchConfiguration('frame_id').perform(context)
    angle_min_deg = float(LaunchConfiguration('angle_min_deg').perform(context))
    angle_max_deg = float(LaunchConfiguration('angle_max_deg').perform(context))
    angle_increment_deg = float(LaunchConfiguration('angle_increment_deg').perform(context))
    range_min = float(LaunchConfiguration('range_min').perform(context))
    range_max = float(LaunchConfiguration('range_max').perform(context))
    publish_rate_hz = float(LaunchConfiguration('publish_rate_hz').perform(context))
    measurement_timeout_sec = float(LaunchConfiguration('measurement_timeout_sec').perform(context))

    raw_node = Node(
        package='sensors_broadcaster',
        executable='multi_vl53_node',
        name='multi_vl53_node',
        output='screen',
        parameters=[{'xshut_pins': xshut_pins}],
    )

    fuser_node = Node(
        package='sensors_broadcaster',
        executable='vl53_scan_fuser_node',
        name='vl53_scan_fuser_node',
        output='screen',
        parameters=[{
            'input_topics': input_topics,
            'sensor_angles_deg': sensor_angles_deg,
            'sensor_fov_deg': sensor_fov_deg,
            'projection_mode': projection_mode,
            'overlap_mode': overlap_mode,
            'output_topic': output_topic,
            'frame_id': frame_id,
            'angle_min_deg': angle_min_deg,
            'angle_max_deg': angle_max_deg,
            'angle_increment_deg': angle_increment_deg,
            'range_min': range_min,
            'range_max': range_max,
            'publish_rate_hz': publish_rate_hz,
            'measurement_timeout_sec': measurement_timeout_sec,
        }],
    )

    return [raw_node, fuser_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'xshut_pins',
            default_value='[4, 17, 27]',
            description='GPIO pins connected to VL53 XSHUT lines.',
        ),
        DeclareLaunchArgument(
            'input_topics',
            default_value="['sensor_1/range', 'sensor_2/range', 'sensor_3/range']",
            description='Raw range topics to fuse into LaserScan.',
        ),
        DeclareLaunchArgument(
            'sensor_angles_deg',
            default_value='[-90.0, 0.0, 90.0]',
            description='Mounting angles of sensors in degrees.',
        ),
        DeclareLaunchArgument(
            'sensor_fov_deg',
            default_value='[27.0]',
            description='FoV for each sensor in degrees; one value can be broadcast to all sensors.',
        ),
        DeclareLaunchArgument(
            'projection_mode',
            default_value='point',
            description='LaserScan projection mode: point or fov.',
        ),
        DeclareLaunchArgument(
            'overlap_mode',
            default_value='average',
            description='How to combine overlapping sensor values: average or min.',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/sensors_broadcaster/scan',
            description='LaserScan output topic.',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='vl53_scan',
            description='frame_id for published LaserScan.',
        ),
        DeclareLaunchArgument(
            'angle_min_deg',
            default_value='-90.0',
            description='LaserScan minimum angle in degrees.',
        ),
        DeclareLaunchArgument(
            'angle_max_deg',
            default_value='90.0',
            description='LaserScan maximum angle in degrees.',
        ),
        DeclareLaunchArgument(
            'angle_increment_deg',
            default_value='1.0',
            description='LaserScan angular increment in degrees.',
        ),
        DeclareLaunchArgument(
            'range_min',
            default_value='0.04',
            description='Minimum valid range in meters.',
        ),
        DeclareLaunchArgument(
            'range_max',
            default_value='4.0',
            description='Maximum valid range in meters.',
        ),
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='20.0',
            description='LaserScan publish rate in Hz.',
        ),
        DeclareLaunchArgument(
            'measurement_timeout_sec',
            default_value='0.25',
            description='Drop stale sensor readings older than this timeout.',
        ),
        OpaqueFunction(function=_build_nodes),
    ])
