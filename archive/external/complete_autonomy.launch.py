#!/usr/bin/env python3
"""
complete_autonomy.launch.py
============================
Launches the full autonomous stack on your PC:
  1. SLAM Toolbox  — builds the map from Ridgeback LiDAR
  2. Simple Wanderer (optional) — drives the robot to explore
  3. Nav2 bringup  — path planning and obstacle avoidance

Usage:
    # SLAM only (manual driving):
    ros2 launch ~/ridgeback/launch/complete_autonomy.launch.py

    # SLAM + auto wanderer:
    ros2 launch ~/ridgeback/launch/complete_autonomy.launch.py wander:=true

    # SLAM + Nav2 (when you need NavigateToPose):
    ros2 launch ~/ridgeback/launch/complete_autonomy.launch.py nav2:=true

Pre-requisites on your PC:
    source /opt/ros/humble/setup.bash
    source ~/ridgeback/install/setup.bash   ← must source this!
    export ROS_DOMAIN_ID=0
    export RMW_FASTRTPS_USE_SHM=0
    export FASTRTPS_DEFAULT_PROFILES_FILE=~/ridgeback/config/fastrtps_jetson.xml
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

THIS_FILE = os.path.abspath(__file__)
DEFAULT_WORKSPACE = os.path.dirname(os.path.dirname(THIS_FILE))
RIDGEBACK_WORKSPACE = os.path.abspath(
    os.path.expanduser(os.environ.get("RIDGEBACK_WORKSPACE", DEFAULT_WORKSPACE))
)
HOME = os.path.expanduser("~")
SLAM_PARAMS = os.path.join(RIDGEBACK_WORKSPACE, "config", "ridgeback_slam_params.yaml")
NAV2_PARAMS = os.path.join(RIDGEBACK_WORKSPACE, "config", "nav2_live_map_params.yaml")
MAP_DIR      = os.path.join(HOME, ".ridgeback")


def generate_launch_description() -> LaunchDescription:
    os.makedirs(MAP_DIR, exist_ok=True)

    # ── arguments ──────────────────────────────────────────────────────────
    wander_arg = DeclareLaunchArgument(
        "wander",
        default_value="false",
        description="Launch simple_wanderer for automatic exploration",
    )
    nav2_arg = DeclareLaunchArgument(
        "nav2",
        default_value="false",
        description="Launch Nav2 navigation stack",
    )

    # ── 1. SLAM Toolbox ────────────────────────────────────────────────────
    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[SLAM_PARAMS],
        remappings=[
            # SLAM Toolbox default topics → Ridgeback-specific topics
            ("scan", "/r100_0140/sensors/lidar2d_0/scan"),
            ("odom", "/r100_0140/platform/odom/filtered"),
        ],
    )

    # ── 2. Simple wanderer (optional automatic exploration) ────────────────
    # Runs simple_wanderer.py from ridgeback_image_motion package.
    # Only active when wander:=true
    wanderer_node = Node(
        package="ridgeback_image_motion",
        executable="simple_wanderer",
        name="simple_wanderer",
        output="screen",
        condition=IfCondition(LaunchConfiguration("wander")),
    )

    # ── 3. Nav2 bringup (optional — needed for NavigateToPose goals) ───────
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_launch = GroupAction(
        condition=IfCondition(LaunchConfiguration("nav2")),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
                ),
                launch_arguments={
                    "params_file": NAV2_PARAMS,
                    "use_sim_time": "false",
                    "autostart": "true",
                }.items(),
            )
        ],
    )

    # ── startup info ───────────────────────────────────────────────────────
    info = [
        LogInfo(msg=["=" * 55]),
        LogInfo(msg=["  Ridgeback Autonomy Stack"]),
        LogInfo(msg=["=" * 55]),
        LogInfo(msg=["  SLAM Toolbox : ACTIVE (async mode)"]),
        LogInfo(msg=["  LiDAR topic  : /r100_0140/sensors/lidar2d_0/scan"]),
        LogInfo(msg=["  Odom topic   : /r100_0140/platform/odom/filtered"]),
        LogInfo(msg=["  Map topic    : /map"]),
        LogInfo(msg=[""]),
        LogInfo(msg=["  To save map manually:"]),
        LogInfo(msg=["    ros2 run nav2_map_server map_saver_cli -f ~/.ridgeback/map"]),
        LogInfo(msg=["  To view map:"]),
        LogInfo(msg=["    rviz2"]),
        LogInfo(msg=["=" * 55]),
    ]

    return LaunchDescription(
        [wander_arg, nav2_arg, slam_node, wanderer_node, nav2_launch] + info
    )
