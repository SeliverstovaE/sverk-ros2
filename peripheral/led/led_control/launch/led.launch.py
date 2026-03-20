#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("led_control")
    default_config = os.path.join(pkg_share, "config", "led_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=default_config,
            description="Path to led_params.yaml (overrides led_count, spi_bus, spi_device from file)",
        ),
        Node(
            package="led_control",
            executable="led_node",
            name="led_node",
            output="screen",
            parameters=[LaunchConfiguration("config")],
        ),
    ])
