#!/usr/bin/env python3
"""
jetson_safety.launch.py
========================
Launches both Jetson safety nodes together.

Usage:
    ros2 launch ridgeback_image_motion jetson_safety.launch.py

Tune at runtime:
    ros2 launch ridgeback_image_motion jetson_safety.launch.py \
        heartbeat_timeout:=1.5 max_linear:=0.3
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    args = [
        DeclareLaunchArgument(
            "heartbeat_timeout", default_value="2.0",
            description="Seconds without heartbeat before e-stop fires",
        ),
        DeclareLaunchArgument(
            "init_grace_period", default_value="5.0",
            description="Startup grace period before watchdog activates (s)",
        ),
        DeclareLaunchArgument(
            "max_linear", default_value="0.5",
            description="Velocity gate: max linear speed m/s",
        ),
        DeclareLaunchArgument(
            "max_angular", default_value="0.5",
            description="Velocity gate: max angular speed rad/s",
        ),
        DeclareLaunchArgument(
            "log_clamps", default_value="true",
            description="Log a warning when velocity gate clamps a value",
        ),
    ]

    watchdog = Node(
        package="ridgeback_image_motion",
        executable="jetson_watchdog",
        name="jetson_watchdog",
        output="screen",
        parameters=[{
            "heartbeat_timeout": LaunchConfiguration("heartbeat_timeout"),
            "init_grace_period": LaunchConfiguration("init_grace_period"),
            "cmd_vel_topic":     "/r100_0140/cmd_vel",
        }],
    )

    velocity_gate = Node(
        package="ridgeback_image_motion",
        executable="velocity_gate",
        name="velocity_gate",
        output="screen",
        parameters=[{
            "max_linear":   LaunchConfiguration("max_linear"),
            "max_angular":  LaunchConfiguration("max_angular"),
            "input_topic":  "/cmd_vel_raw",
            "output_topic": "/r100_0140/cmd_vel",
            "log_clamps":   LaunchConfiguration("log_clamps"),
        }],
    )

    return LaunchDescription(args + [watchdog, velocity_gate])
