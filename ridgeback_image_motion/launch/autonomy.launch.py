"""Jetson-side autonomy stack.

Profiles: full | safety_only | web_only.
Per-component toggles override the profile defaults: enable_slam,
enable_nav2, enable_explorer, enable_vlm, enable_web.

Always-on (every profile): safety_controller, cmd_vel_mux,
jetson_watchdog. SLAM, Nav2, frontier+mission+memory, VLM, and the web
dashboard are conditional includes. The Ridgeback-side ridgeback_start.sh
runs motion_server.py to bridge /cmd_vel/mux_out into Clearpath's native
twist_mux input.
"""

from __future__ import annotations

import os
import shutil
import subprocess

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _detect_realsense_usb_speed() -> str:
    """Return '3' (USB3) or '2' (USB2) for an attached Intel D435.

    Probes lsusb -t. If lsusb or the camera is absent, falls back to '2' so
    the conservative profile is used (works on any speed but lower quality).
    """
    lsusb = shutil.which("lsusb")
    if lsusb is None:
        return "2"
    try:
        proc = subprocess.run([lsusb, "-t"], capture_output=True, text=True, timeout=2.0)
    except Exception:
        return "2"
    lines = proc.stdout.splitlines()
    video_lines = [
        line for line in lines if "uvcvideo" in line or "Class=Video" in line
    ]
    scan_lines = video_lines or lines
    for marker in ("10000M", "5000M"):
        if any(marker in line for line in scan_lines):
            return "3"
    if any("480M" in line for line in scan_lines):
        return "2"
    return "2"


def _realsense_launch_args(camera_namespace: str, camera_name: str) -> tuple[str, dict[str, str]]:
    """Pick rs_launch.py args for the detected USB speed.

    Returns (profile_label, args_dict).
    """
    speed = _detect_realsense_usb_speed()
    # IR streams are disabled — nothing downstream consumes them, and the
    # D435 defaults of 848x480@30 saturate the Jetson USB bus enough to
    # trip the "Frames didn't arrive within 5 seconds" watchdog.
    if speed == "3":
        return "USB3 (424x240 RGB+depth @15, IR off)", {
            "camera_namespace": camera_namespace,
            "camera_name": camera_name,
            "enable_color": "true",
            "rgb_camera.color_profile": "424x240x15",
            "enable_depth": "true",
            "depth_module.depth_profile": "424x240x15",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "align_depth.enable": "true",
            "enable_sync": "false",
            "pointcloud.enable": "false",
        }
    return "USB2 (424x240 color-only @15, IR off)", {
        "camera_namespace": camera_namespace,
        "camera_name": camera_name,
        "enable_color": "true",
        "rgb_camera.color_profile": "424x240x15",
        "enable_depth": "false",
        "enable_infra1": "false",
        "enable_infra2": "false",
        "align_depth.enable": "false",
        "pointcloud.enable": "false",
    }


def _resolve_toggle(context, name: str, profile_truth_map: dict[str, bool]) -> str:
    profile = LaunchConfiguration("profile").perform(context)
    explicit = LaunchConfiguration(name).perform(context)
    if explicit in ("true", "false"):
        return explicit
    # auto -> derive from profile
    return "true" if profile_truth_map.get(profile, False) else "false"


def _nav2_node(
    package: str,
    executable: str,
    name: str,
    params_file: str,
    extra_remappings: list[tuple[str, str]] | None = None,
    extra_parameters: list | None = None,
) -> Node:
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    if extra_remappings:
        remappings.extend(extra_remappings)
    parameters: list = [params_file]
    if extra_parameters:
        parameters.extend(extra_parameters)
    return Node(
        package=package,
        executable=executable,
        name=name,
        output="screen",
        parameters=parameters,
        remappings=remappings,
        emulate_tty=True,
    )


def _setup(context, *args, **kwargs):
    pkg = FindPackageShare("ridgeback_image_motion")
    params_file = LaunchConfiguration("params_file").perform(context)
    slam_params_file = LaunchConfiguration("slam_params_file").perform(context)
    nav2_params_file = LaunchConfiguration("nav2_params_file").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    web_host = LaunchConfiguration("host").perform(context)
    web_port = LaunchConfiguration("port").perform(context)

    full = {"full": True, "safety_only": False, "web_only": False}
    web = {"full": True, "safety_only": False, "web_only": True}
    # Camera defaults to on in full and web_only (dashboard needs the feed).
    cam_truth = {"full": True, "safety_only": False, "web_only": True}
    enable_slam = _resolve_toggle(context, "enable_slam", full)
    enable_nav2 = _resolve_toggle(context, "enable_nav2", full)
    enable_explorer = _resolve_toggle(context, "enable_explorer", full)
    enable_vlm = _resolve_toggle(context, "enable_vlm", full)
    enable_web = _resolve_toggle(context, "enable_web", web)
    enable_camera = _resolve_toggle(context, "enable_camera", cam_truth)

    common_params = [params_file, {"namespace": namespace}]

    actions = [
        LogInfo(msg=[
            "[autonomy] profile=", LaunchConfiguration("profile"),
            " slam=", enable_slam,
            " nav2=", enable_nav2,
            " explorer=", enable_explorer,
            " vlm=", enable_vlm,
            " web=", enable_web,
            " camera=", enable_camera,
        ]),

        # Always-on: safety / mux / watchdog. The Ridgeback-side script owns
        # motion_server.py so there is exactly one platform command publisher.
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

    # The platform publishes TFs on /r100_0140/tf, but slam_toolbox/nav2 run
    # at root and subscribe to /tf. Bridge with our own node that uses
    # explicit QoS — topic_tools relay has races with /tf_static's
    # transient_local durability that silently drop frames into SLAM.
    if enable_slam == "true" or enable_nav2 == "true":
        actions.append(
            Node(
                package="ridgeback_image_motion",
                executable="tf_bridge.py",
                name="tf_bridge",
                output="screen",
                parameters=[{
                    "source_tf": f"/{namespace}/tf",
                    "source_tf_static": f"/{namespace}/tf_static",
                    "output_tf": "/tf",
                    "output_tf_static": "/tf_static",
                }],
                emulate_tty=True,
            )
        )

    if enable_slam == "true":
        slam_pkg = FindPackageShare("slam_toolbox").perform(context)
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    f"{slam_pkg}/launch/online_async_launch.py"
                ),
                launch_arguments={
                    # nav2/slam launch files build PythonExpression([... use_sim_time])
                    # which evaluates the literal as Python — use capital True/False.
                    "use_sim_time": "False",
                    "slam_params_file": slam_params_file,
                }.items(),
            )
        )

    if enable_nav2 == "true":
        try:
            nav2_pkg = FindPackageShare("nav2_bringup").perform(context)
        except Exception as exc:
            actions.append(
                LogInfo(msg=[f"[autonomy] ERROR: nav2_bringup not found ({exc})"])
            )
            nav2_pkg = ""
        if nav2_pkg:
            actions.append(
                LogInfo(msg=[
                    "[autonomy] launching Nav2 nodes directly with params: ",
                    nav2_params_file,
                ])
            )
            # Custom short-backup BT lives next to nav2_params.yaml in the
            # workspace config/ dir (same dir nav2_params_file is read from).
            # Empty string -> bt_navigator keeps Nav2's stock tree.
            _short_backup_bt = os.path.join(
                os.path.dirname(nav2_params_file), "nav2_bt_short_backup.xml"
            )
            if os.path.isfile(_short_backup_bt):
                actions.append(LogInfo(msg=[
                    "[autonomy] NavigateToPose BT overridden (short backup): ",
                    _short_backup_bt,
                ]))
            else:
                actions.append(LogInfo(msg=[
                    "[autonomy] WARN short-backup BT not found, using Nav2 default: ",
                    _short_backup_bt,
                ]))
                _short_backup_bt = ""
            # Avoid nav2_bringup's RewrittenYaml temp-file layer here. On the
            # Jetson it was producing a /tmp/launch_params_* file that behaved
            # like the default Nav2 config (goal_checker/static_layer/no DWB
            # critics) despite the logged source YAML being correct.
            nav2_actions = [
                _nav2_node(
                    "nav2_controller",
                    "controller_server",
                    "controller_server",
                    nav2_params_file,
                    extra_remappings=[("cmd_vel", "/cmd_vel/nav")],
                ),
                _nav2_node(
                    "nav2_smoother",
                    "smoother_server",
                    "smoother_server",
                    nav2_params_file,
                ),
                _nav2_node(
                    "nav2_planner",
                    "planner_server",
                    "planner_server",
                    nav2_params_file,
                ),
                _nav2_node(
                    "nav2_behaviors",
                    "behavior_server",
                    "behavior_server",
                    nav2_params_file,
                    # Without this remap, behavior_server's recovery
                    # commands (spin/backup/drive_on_heading) publish to
                    # /cmd_vel, which cmd_vel_mux does not subscribe to,
                    # so the robot never executes any recovery. The
                    # behavior_server runs each behavior to its timeout
                    # without the robot moving an inch.
                    extra_remappings=[("cmd_vel", "/cmd_vel/nav")],
                ),
                _nav2_node(
                    "nav2_bt_navigator",
                    "bt_navigator",
                    "bt_navigator",
                    nav2_params_file,
                    # Override the NavigateToPose tree with our short-backup
                    # variant (BackUp 0.10 m instead of 0.30 m -- the rear is a
                    # LiDAR blind spot, see the XML header). Resolved next to
                    # nav2_params.yaml so there's no hardcoded home path. Falls
                    # back to Nav2's stock tree if the file is missing, so a bad
                    # pull can't take the whole nav stack down.
                    extra_parameters=(
                        [{"default_nav_to_pose_bt_xml": _short_backup_bt}]
                        if _short_backup_bt else None
                    ),
                ),
                _nav2_node(
                    "nav2_waypoint_follower",
                    "waypoint_follower",
                    "waypoint_follower",
                    nav2_params_file,
                ),
                _nav2_node(
                    "nav2_velocity_smoother",
                    "velocity_smoother",
                    "velocity_smoother",
                    nav2_params_file,
                ),
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_navigation",
                    output="screen",
                    parameters=[
                        nav2_params_file,
                        {"use_sim_time": False, "autostart": True},
                    ],
                    emulate_tty=True,
                ),
            ]
            # Let the /r100_0140/tf -> /tf bridge and slam_toolbox start
            # publishing before lifecycle_manager activates costmaps.
            actions.append(TimerAction(period=3.0, actions=nav2_actions))

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
            # Doorway/gap fit gate. Jetson-side only (consumes the bridged scan
            # + odom); feeds /frontier/blacklist_point so the explorer reroutes
            # away from openings the robot doesn't fit through.
            Node(
                package="ridgeback_image_motion",
                executable="passage_monitor.py",
                name="passage_monitor",
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

    if enable_camera == "true":
        profile_label, rs_args = _realsense_launch_args(
            camera_namespace=f"/{namespace}/sensors",
            camera_name="camera_0",
        )
        # Direct Node action instead of IncludeLaunchDescription(rs_launch.py).
        # rs_launch.py greedily reads ALL parent LaunchConfigurations and
        # passes them to the realsense node, producing pages of "Parameter
        # 'host'/'enable_slam'/etc. is not supported" warnings. The direct
        # form passes only the params we set.
        rs_params = {
            k: (v == "true") if v in ("true", "false") else v
            for k, v in rs_args.items()
            if k not in ("camera_namespace", "camera_name")
        }
        actions.append(
            LogInfo(msg=[f"[autonomy] RealSense profile: {profile_label}"])
        )
        actions.append(
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                namespace=rs_args["camera_namespace"],
                name=rs_args["camera_name"],
                parameters=[rs_params],
                output="screen",
                emulate_tty=True,
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare("ridgeback_image_motion")
    default_params = PathJoinSubstitution([pkg, "config", "autonomy_params.yaml"])
    default_slam_params = PathJoinSubstitution([pkg, "config", "slam_params.yaml"])
    default_nav2_params = PathJoinSubstitution([pkg, "config", "nav2_params.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "profile", default_value="full",
            description="full | safety_only | web_only",
        ),
        DeclareLaunchArgument("namespace", default_value="r100_0140"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("slam_params_file", default_value=default_slam_params),
        DeclareLaunchArgument("nav2_params_file", default_value=default_nav2_params),
        DeclareLaunchArgument("host", default_value="0.0.0.0"),
        DeclareLaunchArgument("port", default_value="8081"),
        DeclareLaunchArgument("enable_slam", default_value="auto"),
        DeclareLaunchArgument("enable_nav2", default_value="auto"),
        DeclareLaunchArgument("enable_explorer", default_value="auto"),
        DeclareLaunchArgument("enable_vlm", default_value="auto"),
        DeclareLaunchArgument("enable_web", default_value="auto"),
        DeclareLaunchArgument("enable_camera", default_value="auto"),
        OpaqueFunction(function=_setup),
    ])
