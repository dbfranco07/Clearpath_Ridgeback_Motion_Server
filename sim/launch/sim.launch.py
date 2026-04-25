"""Simulation launch wrapper for the Ridgeback autonomy stack.

Gazebo/ros-ign-bridge provide the same /r100_0140 sensor topics used on
hardware. This file intentionally includes the production autonomy launch so
sim tests exercise the same mission, safety, SLAM, Nav2, and dashboard path.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    profile = LaunchConfiguration("profile")
    launch_vlm = LaunchConfiguration("launch_vlm")
    launch_vslam = LaunchConfiguration("launch_vslam")

    autonomy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ridgeback_image_motion"), "launch", "autonomy.launch.py"]
            )
        ),
        launch_arguments={
            "profile": profile,
            "launch_vlm": launch_vlm,
            "launch_vslam": launch_vslam,
            "host": "0.0.0.0",
            "port": "8081",
        }.items(),
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[{"port": 8765, "address": "0.0.0.0", "use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("profile", default_value="debug"),
            DeclareLaunchArgument("launch_vlm", default_value="false"),
            DeclareLaunchArgument("launch_vslam", default_value="false"),
            autonomy_launch,
            foxglove_bridge,
        ]
    )
