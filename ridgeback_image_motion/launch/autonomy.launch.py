"""Jetson-side autonomy stack.

Profiles: full | safety_only | web_only.
Per-component toggles override the profile defaults: enable_slam,
enable_nav2, enable_explorer, enable_vlm, enable_web.

Always-on (every profile): safety_controller, cmd_vel_mux,
jetson_watchdog. SLAM, Nav2, frontier+mission+memory, VLM, and the web
dashboard are conditional includes.
"""

from __future__ import annotations

import shutil
import subprocess

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
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
    target_line = ""
    for line in proc.stdout.splitlines():
        # The D435 advertises Intel 0x8086:0x0b07 (and similar variants).
        if "Intel" in line or "8086:0b07" in line:
            target_line = line
            break
    for marker, code in (("10000M", "3"), ("5000M", "3"), ("480M", "2")):
        if marker in target_line:
            return code
    return "2"


def _realsense_launch_args(camera_namespace: str, camera_name: str) -> tuple[str, dict[str, str]]:
    """Pick rs_launch.py args for the detected USB speed.

    Returns (profile_label, args_dict).
    """
    speed = _detect_realsense_usb_speed()
    if speed == "3":
        return "USB3 (640x480 RGB+depth @30)", {
            "camera_namespace": camera_namespace,
            "camera_name": camera_name,
            "enable_color": "true",
            "rgb_camera.color_profile": "640x480x30",
            "enable_depth": "true",
            "depth_module.depth_profile": "640x480x30",
            "align_depth.enable": "true",
            "enable_sync": "true",
            "pointcloud.enable": "false",
        }
    return "USB2 (424x240 color-only @15)", {
        "camera_namespace": camera_namespace,
        "camera_name": camera_name,
        "enable_color": "true",
        "rgb_camera.color_profile": "424x240x15",
        "enable_depth": "false",
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


def _setup(context, *args, **kwargs):
    pkg = FindPackageShare("ridgeback_image_motion")
    params_file = LaunchConfiguration("params_file").perform(context)
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
        try:
            nav2_pkg = FindPackageShare("nav2_bringup").perform(context)
        except Exception as exc:
            actions.append(
                LogInfo(msg=[f"[autonomy] ERROR: nav2_bringup not found ({exc})"])
            )
            nav2_pkg = ""
        if nav2_pkg:
            nav2_launch = f"{nav2_pkg}/launch/navigation_launch.py"
            nav2_params = PathJoinSubstitution([pkg, "config", "nav2_params.yaml"]).perform(context)
            actions.append(
                LogInfo(msg=[f"[autonomy] including {nav2_launch}"])
            )
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch),
                    launch_arguments={
                        "use_sim_time": "False",
                        "params_file": nav2_params,
                        "autostart": "True",
                        # Orin Nano can't carry 7 separate Nav2 processes
                        # plus SLAM/VLM/camera/web — DDS service discovery
                        # times out under load. Composition puts every
                        # Nav2 server in one process.
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
        DeclareLaunchArgument("enable_camera", default_value="auto"),
        OpaqueFunction(function=_setup),
    ])
