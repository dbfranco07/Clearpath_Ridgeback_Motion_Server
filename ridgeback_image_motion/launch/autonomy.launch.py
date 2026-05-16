"""Jetson-side autonomy stack.

Profiles: full | safety_only | web_only.
Per-component toggles override the profile defaults: enable_slam,
enable_nav2, enable_explorer, enable_vlm, enable_web.

Always-on (every profile): safety_controller, cmd_vel_mux,
jetson_watchdog. SLAM, Nav2, frontier+mission+memory, VLM, and the web
dashboard are conditional includes.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _resolve_toggle(context, name: str, profile_truth_map: dict[str, bool]) -> str:
    profile = LaunchConfiguration("profile").perform(context)
    explicit = LaunchConfiguration(name).perform(context)
    if explicit in ("true", "false"):
        return explicit
    # auto -> derive from profile
    return "true" if profile_truth_map.get(profile, False) else "false"


def _setup(context, *args, **kwargs):
    pkg = FindPackageShare("ridgeback_image_motion")
    params_file = LaunchConfiguration("params_file").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    web_host = LaunchConfiguration("host").perform(context)
    web_port = LaunchConfiguration("port").perform(context)

    full = {"full": True, "safety_only": False, "web_only": False}
    web = {"full": True, "safety_only": False, "web_only": True}
    enable_slam = _resolve_toggle(context, "enable_slam", full)
    enable_nav2 = _resolve_toggle(context, "enable_nav2", full)
    enable_explorer = _resolve_toggle(context, "enable_explorer", full)
    enable_vlm = _resolve_toggle(context, "enable_vlm", full)
    enable_web = _resolve_toggle(context, "enable_web", web)

    common_params = [params_file, {"namespace": namespace}]

    actions = [
        LogInfo(msg=[
            "[autonomy] profile=", LaunchConfiguration("profile"),
            " slam=", enable_slam,
            " nav2=", enable_nav2,
            " explorer=", enable_explorer,
            " vlm=", enable_vlm,
            " web=", enable_web,
        ]),

        # Always-on: safety / mux / watchdog.
        Node(
            package="ridgeback_image_motion",
            executable="safety_controller.py",
            name="safety_controller",
            output="screen",
            parameters=common_params,
            emulate_tty=True,
        ),
        Node(
            package="ridgeback_image_motion",
            executable="cmd_vel_mux.py",
            name="cmd_vel_mux",
            output="screen",
            parameters=common_params,
            emulate_tty=True,
        ),
        Node(
            package="ridgeback_image_motion",
            executable="jetson_watchdog.py",
            name="jetson_watchdog",
            output="screen",
            parameters=common_params,
            emulate_tty=True,
        ),
    ]

    if enable_web == "true":
        actions.append(
            Node(
                package="ridgeback_image_motion",
                executable="web_dashboard.py",
                name="ridgeback_dashboard",
                output="screen",
                arguments=["--host", web_host, "--port", web_port],
                parameters=common_params,
                emulate_tty=True,
            )
        )

    if enable_slam == "true":
        slam_pkg = FindPackageShare("slam_toolbox").perform(context)
        slam_params = PathJoinSubstitution([pkg, "config", "slam_params.yaml"]).perform(context)
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    f"{slam_pkg}/launch/online_async_launch.py"
                ),
                launch_arguments={
                    # nav2/slam launch files build PythonExpression([... use_sim_time])
                    # which evaluates the literal as Python — use capital True/False.
                    "use_sim_time": "False",
                    "slam_params_file": slam_params,
                }.items(),
            )
        )

    if enable_nav2 == "true":
        nav2_pkg = FindPackageShare("nav2_bringup").perform(context)
        nav2_params = PathJoinSubstitution([pkg, "config", "nav2_params.yaml"]).perform(context)
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    f"{nav2_pkg}/launch/navigation_launch.py"
                ),
                launch_arguments={
                    "use_sim_time": "False",
                    "params_file": nav2_params,
                    "autostart": "True",
                    "use_composition": "True",
                }.items(),
            )
        )

    if enable_explorer == "true":
        actions.extend([
            Node(
                package="ridgeback_image_motion",
                executable="frontier_explorer.py",
                name="frontier_explorer",
                output="screen",
                parameters=common_params,
                emulate_tty=True,
            ),
            Node(
                package="ridgeback_image_motion",
                executable="mission_orchestrator.py",
                name="mission_orchestrator",
                output="screen",
                parameters=common_params,
                emulate_tty=True,
            ),
            Node(
                package="ridgeback_image_motion",
                executable="spatial_memory.py",
                name="spatial_memory",
                output="screen",
                parameters=common_params,
                emulate_tty=True,
            ),
        ])

    if enable_vlm == "true":
        actions.append(
            Node(
                package="ridgeback_image_motion",
                executable="vlm_client.py",
                name="vlm_client",
                output="screen",
                parameters=common_params,
                emulate_tty=True,
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare("ridgeback_image_motion")
    default_params = PathJoinSubstitution([pkg, "config", "autonomy_params.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "profile", default_value="full",
            description="full | safety_only | web_only",
        ),
        DeclareLaunchArgument("namespace", default_value="r100_0140"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("host", default_value="0.0.0.0"),
        DeclareLaunchArgument("port", default_value="8081"),
        DeclareLaunchArgument("enable_slam", default_value="auto"),
        DeclareLaunchArgument("enable_nav2", default_value="auto"),
        DeclareLaunchArgument("enable_explorer", default_value="auto"),
        DeclareLaunchArgument("enable_vlm", default_value="auto"),
        DeclareLaunchArgument("enable_web", default_value="auto"),
        OpaqueFunction(function=_setup),
    ])
