"""
sim.launch.py — launches the ROS2 autonomy stack in simulation mode.

Called by docker-compose autonomy container. Differences from hw.launch.py:
  - No hardware topic sources (Gazebo provides sensors via ros-ign-bridge)
  - SIM_MODE=true env var set (autonomy nodes detect this for minor tweaks)
  - spawn_entity handles Ridgeback model lifecycle (not clearpath-robot.service)
  - foxglove_bridge included for MacBook visualization

When the autonomy package is partially implemented, comment out nodes not
yet written and reintroduce them as they are built.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    strategy_arg = DeclareLaunchArgument(
        "strategy",
        default_value=os.getenv("STRATEGY", "frontier_nearest"),
        description="Exploration strategy to use",
    )
    vlm_endpoint_arg = DeclareLaunchArgument(
        "vlm_endpoint",
        default_value=os.getenv(
            "VLM_ENDPOINT", "http://10.158.36.90:8000/v1/chat/completions"
        ),
        description="vLLM endpoint URL",
    )

    strategy = LaunchConfiguration("strategy")
    vlm_endpoint = LaunchConfiguration("vlm_endpoint")

    # ── SLAM toolbox (async mode, lower update rate for Jetson Nano) ──
    # TODO: wire in once slam_toolbox params are tuned
    # slam_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         get_package_share_directory("slam_toolbox"),
    #         "/launch/online_async_launch.py"
    #     ]),
    #     launch_arguments={
    #         "slam_params_file": "/ros2_ws/src/ridgeback_image_motion/config/slam_params.yaml",
    #         "use_sim_time": "true",
    #     }.items(),
    # )

    # ── Nav2 ──────────────────────────────────────────────────────────
    # TODO: wire in once nav2 params are ready
    # nav2 = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         get_package_share_directory("nav2_bringup"), "/launch/navigation_launch.py"
    #     ]),
    #     launch_arguments={
    #         "params_file": "/ros2_ws/src/ridgeback_image_motion/config/nav2_params.yaml",
    #         "use_sim_time": "true",
    #     }.items(),
    # )

    # ── Foxglove bridge (visualization on MacBook) ────────────────────
    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[{
            "port": 8765,
            "address": "0.0.0.0",
            "use_sim_time": True,
        }],
        output="screen",
    )

    # ── Safety controller ─────────────────────────────────────────────
    # TODO: uncomment when safety_controller.py is implemented
    # safety_controller = Node(
    #     package="ridgeback_image_motion",
    #     executable="safety_controller.py",
    #     name="safety_controller",
    #     parameters=[{
    #         "use_sim_time": True,
    #         "stop_distance_m": 0.4,
    #         "watchdog_timeout_s": 3.0,
    #     }],
    #     output="screen",
    # )

    # ── cmd_vel mux ───────────────────────────────────────────────────
    # TODO: uncomment when cmd_vel_mux.py is implemented
    # cmd_vel_mux = Node(
    #     package="ridgeback_image_motion",
    #     executable="cmd_vel_mux.py",
    #     name="cmd_vel_mux",
    #     parameters=[{"use_sim_time": True}],
    #     output="screen",
    # )

    # ── VLM client + room detector ────────────────────────────────────
    # TODO: uncomment when vlm_client.py and room_detector.py are implemented
    # vlm_client = Node(
    #     package="ridgeback_image_motion",
    #     executable="vlm_client.py",
    #     name="vlm_client",
    #     parameters=[{
    #         "endpoint": vlm_endpoint,
    #         "use_sim_time": True,
    #         "prompts_file": "/ros2_ws/src/ridgeback_image_motion/config/prompts.yaml",
    #     }],
    #     output="screen",
    # )

    # ── Spatial memory ────────────────────────────────────────────────
    # TODO: uncomment when spatial_memory.py is implemented
    # spatial_memory = Node(
    #     package="ridgeback_image_motion",
    #     executable="spatial_memory.py",
    #     name="spatial_memory",
    #     parameters=[{
    #         "db_path": "/tmp/sim_memory.db",
    #         "use_sim_time": True,
    #     }],
    #     output="screen",
    # )

    # ── Mission orchestrator ──────────────────────────────────────────
    # TODO: uncomment when mission_orchestrator.py is implemented
    # mission_orchestrator = Node(
    #     package="ridgeback_image_motion",
    #     executable="mission_orchestrator.py",
    #     name="mission_orchestrator",
    #     parameters=[{
    #         "strategy": strategy,
    #         "use_sim_time": True,
    #         "goal_tolerance_m": 0.5,
    #     }],
    #     output="screen",
    # )

    # ── Dashboard ─────────────────────────────────────────────────────
    # TODO: uncomment when dashboard.py is implemented
    # For now, use the existing web_controller.py as a placeholder
    # (it shows camera + LiDAR but no mission control)
    # dashboard = Node(
    #     package="ridgeback_image_motion",
    #     executable="dashboard.py",
    #     name="dashboard",
    #     parameters=[{"port": 8081, "use_sim_time": True}],
    #     output="screen",
    # )

    return LaunchDescription([
        strategy_arg,
        vlm_endpoint_arg,
        # ── Active nodes ─────────────────────────────────────────────
        foxglove_bridge,
        # Uncomment nodes above as they are implemented.
        # Current "hello world" state:
        #   - Foxglove bridge: up, lets you visualize topics from Gazebo
        #   - Dashboard: TODO
        #   - All autonomy nodes: TODO
    ])
