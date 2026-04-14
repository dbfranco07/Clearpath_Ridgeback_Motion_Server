"""
Simulation Launch — MacBook M4 / Desktop
==========================================
Runs the full autonomy stack against a Gazebo simulation.
Use Docker: osrf/ros:humble-desktop with Gazebo Fortress/Classic.

Robot: TurtleBot3 Burger (differential drive, approximates Ridgeback for
logic testing) OR a custom holonomic model if available.

Topic bridge: simulated topics are remapped to match Ridgeback's namespace
so all autonomy nodes work unchanged.

Usage (in Docker or native ROS2):
  ros2 launch ridgeback_autonomy simulation.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    SetEnvironmentVariable, TimerAction, LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg = FindPackageShare('ridgeback_autonomy')
    world_file = PathJoinSubstitution([pkg, 'worlds', 'office_floor.world'])

    safety_params = PathJoinSubstitution([pkg, 'config', 'safety_params.yaml'])
    slam_params   = PathJoinSubstitution([pkg, 'config', 'slam_toolbox_params.yaml'])
    nav2_params   = PathJoinSubstitution([pkg, 'config', 'nav2_params.yaml'])
    vlm_config    = PathJoinSubstitution([pkg, 'config', 'vlm_config.yaml'])

    # For TurtleBot3 simulation
    tb3_model = LaunchConfiguration('tb3_model')

    return LaunchDescription([
        # ── Arguments ───────────────────────────────────────────
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('tb3_model', default_value='burger',
                              description='TurtleBot3 model: burger, waffle, waffle_pi'),
        DeclareLaunchArgument('world', default_value=world_file),
        DeclareLaunchArgument('mock_vlm', default_value='true',
                              description='Use mock VLM instead of real VLLM server'),

        SetEnvironmentVariable('TURTLEBOT3_MODEL', tb3_model),

        LogInfo(msg='========================================'),
        LogInfo(msg='Ridgeback Autonomy — SIMULATION MODE'),
        LogInfo(msg='========================================'),

        # ── Gazebo ──────────────────────────────────────────────
        # Launch Gazebo with office world
        # NOTE: Requires gazebo_ros package installed
        # ros2 launch gazebo_ros gazebo.launch.py world:=<world_file>
        # Uncomment when Gazebo is available:
        #
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        #     ]),
        #     launch_arguments={'world': LaunchConfiguration('world')}.items()
        # ),

        # ── Topic Bridge ─────────────────────────────────────────
        # Remap simulated TurtleBot3 topics to match Ridgeback namespace
        # so all autonomy nodes work without changes
        Node(
            package='ridgeback_autonomy',
            executable='sim_topic_bridge.py',
            name='sim_topic_bridge',
            output='screen',
            emulate_tty=True
        ),

        # ── Safety ──────────────────────────────────────────────
        Node(
            package='ridgeback_autonomy',
            executable='safety_controller.py',
            name='safety_controller',
            parameters=[
                safety_params,
                {'use_sim_time': True}
            ],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='ridgeback_autonomy',
            executable='cmd_vel_mux.py',
            name='cmd_vel_mux',
            parameters=[
                safety_params,
                {'use_sim_time': True,
                 'output_topic': '/cmd_vel'}  # In sim, output to /cmd_vel directly
            ],
            output='screen',
            emulate_tty=True
        ),

        # ── SLAM + Nav2 ──────────────────────────────────────────
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([pkg, 'launch', 'slam_nav.launch.py'])
                    ]),
                    launch_arguments={
                        'use_sim_time': 'true',
                        'slam_params': slam_params,
                        'nav2_params': nav2_params,
                    }.items()
                ),
            ]
        ),

        # ── VLM + Memory ─────────────────────────────────────────
        TimerAction(
            period=6.0,
            actions=[
                # Use mock VLM in simulation (script below)
                Node(
                    package='ridgeback_autonomy',
                    executable='vlm_perception.py',
                    name='vlm_perception',
                    parameters=[
                        vlm_config,
                        {'use_sim_time': True,
                         'vllm_endpoint': 'http://localhost:9999/v1/chat/completions'}
                    ],
                    output='screen',
                    emulate_tty=True
                ),
                Node(
                    package='ridgeback_autonomy',
                    executable='spatial_memory.py',
                    name='spatial_memory',
                    parameters=[vlm_config, {'use_sim_time': True}],
                    output='screen',
                    emulate_tty=True
                ),
            ]
        ),

        # ── Orchestrator + Dashboard ─────────────────────────────
        TimerAction(
            period=9.0,
            actions=[
                Node(
                    package='ridgeback_autonomy',
                    executable='mission_orchestrator.py',
                    name='mission_orchestrator',
                    parameters=[vlm_config, {'use_sim_time': True}],
                    output='screen',
                    emulate_tty=True
                ),
                Node(
                    package='ridgeback_autonomy',
                    executable='web_dashboard.py',
                    name='web_dashboard',
                    output='screen',
                    emulate_tty=True
                ),
            ]
        ),
    ])
