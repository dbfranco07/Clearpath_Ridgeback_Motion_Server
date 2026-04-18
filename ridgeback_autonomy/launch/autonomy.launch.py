"""
Full Autonomy Stack Launch
===========================
Launches all components:
  1. Safety Controller + Cmd Vel Mux
  2. SLAM Toolbox + Nav2
  3. VLM Perception Node
  4. Spatial Memory Node
  5. Mission Orchestrator
  6. Autonomous Web Dashboard (port 8081)

Prerequisites:
  - Ridgeback clearpath-robot.service running
  - VLLM server accessible at configured endpoint
  - ROS_DOMAIN_ID=0 on both Ridgeback and Jetson
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, LogInfo
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('ridgeback_autonomy')

    safety_params = PathJoinSubstitution([pkg, 'config', 'safety_params.yaml'])
    slam_params   = PathJoinSubstitution([pkg, 'config', 'slam_toolbox_params.yaml'])
    nav2_params   = PathJoinSubstitution([pkg, 'config', 'nav2_params.yaml'])
    vlm_config    = PathJoinSubstitution([pkg, 'config', 'vlm_config.yaml'])

    return LaunchDescription([
        # ── Arguments ──────────────────────────────────────────────────────
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('launch_slam', default_value='true',
                              description='Launch SLAM Toolbox'),
        DeclareLaunchArgument('launch_nav2', default_value='true',
                              description='Launch Nav2 navigation stack'),
        DeclareLaunchArgument('launch_vlm', default_value='true',
                              description='Launch VLM perception node'),
        DeclareLaunchArgument('launch_dashboard', default_value='true',
                              description='Launch web dashboard on port 8081'),
        DeclareLaunchArgument('launch_memory', default_value='true',
                      description='Launch spatial memory node'),
        DeclareLaunchArgument('launch_mission', default_value='true',
                      description='Launch mission orchestrator'),

        LogInfo(msg='========================================'),
        LogInfo(msg='Ridgeback Autonomy — Full Stack Launch'),
        LogInfo(msg='========================================'),

        # ── Tier 1: Safety (launch immediately) ───────────────────────────
        Node(
            package='ridgeback_autonomy',
            executable='safety_controller.py',
            name='safety_controller',
            parameters=[safety_params],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='ridgeback_autonomy',
            executable='cmd_vel_mux.py',
            name='cmd_vel_mux',
            parameters=[safety_params],
            output='screen',
            emulate_tty=True
        ),

        # ── Tier 2: SLAM Toolbox (after 2s — let safety come up first) ────
        TimerAction(
            period=2.0,
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('launch_slam'), "' == 'true' or '",
                LaunchConfiguration('launch_nav2'), "' == 'true'"
            ])),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([pkg, 'launch', 'slam_nav.launch.py'])
                    ]),
                    launch_arguments={
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'slam_params': slam_params,
                        'nav2_params': nav2_params,
                        'launch_slam': LaunchConfiguration('launch_slam'),
                        'launch_nav2': LaunchConfiguration('launch_nav2'),
                    }.items()
                )
            ]
        ),

        # ── Tier 3: Perception + Memory (after 5s — let SLAM warm up) ─────
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ridgeback_autonomy',
                    executable='vlm_perception.py',
                    name='vlm_perception',
                    condition=IfCondition(LaunchConfiguration('launch_vlm')),
                    parameters=[vlm_config],
                    output='screen',
                    emulate_tty=True
                ),
                Node(
                    package='ridgeback_autonomy',
                    executable='spatial_memory.py',
                    name='spatial_memory',
                    condition=IfCondition(LaunchConfiguration('launch_memory')),
                    parameters=[vlm_config],
                    output='screen',
                    emulate_tty=True
                ),
            ]
        ),

        # ── Tier 4: Mission Orchestrator (after 8s — Nav2 needs to be up) ─
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='ridgeback_autonomy',
                    executable='mission_orchestrator.py',
                    name='mission_orchestrator',
                    condition=IfCondition(PythonExpression([
                        "'", LaunchConfiguration('launch_mission'), "' == 'true' and '",
                        LaunchConfiguration('launch_nav2'), "' == 'true'"
                    ])),
                    parameters=[vlm_config],
                    output='screen',
                    emulate_tty=True
                ),
            ]
        ),

        # ── Tier 5: Web Dashboard (last — everything else must be up) ──────
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='ridgeback_autonomy',
                    executable='web_dashboard.py',
                    name='web_dashboard',
                    condition=IfCondition(LaunchConfiguration('launch_dashboard')),
                    output='screen',
                    emulate_tty=True
                ),
            ]
        ),
    ])
