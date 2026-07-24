#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    config = LaunchConfiguration("config").perform(context)
    spi_bus = LaunchConfiguration("spi_bus").perform(context)
    spi_device = LaunchConfiguration("spi_device").perform(context)

    parameters = [config]
    # Empty string = caller didn't override -> fall back to whatever config has.
    overrides = {}
    if spi_bus != "":
        overrides["spi_bus"] = int(spi_bus)
    if spi_device != "":
        overrides["spi_device"] = int(spi_device)
    if overrides:
        parameters.append(overrides)

    return [
        Node(
            package="led_control",
            executable="led_node",
            name="led_node",
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("led_control")
    default_config = os.path.join(pkg_share, "config", "led_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=default_config,
            description="Path to led_params.yaml (overrides led_count, spi_bus, spi_device from file)",
        ),
        DeclareLaunchArgument(
            "spi_bus",
            default_value="",
            description="Overrides spi_bus from config if non-empty (per-board value, see full_system_real.launch.py -> LED_SPI)",
        ),
        DeclareLaunchArgument(
            "spi_device",
            default_value="",
            description="Overrides spi_device from config if non-empty",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
