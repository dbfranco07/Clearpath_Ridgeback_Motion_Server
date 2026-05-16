"""Ridgeback-side launch: motion_server only.

The RealSense camera is brought up by ``scripts/ridgeback_start.sh``
after probing USB speed (USB2 vs USB3) so this launch file stays
parameter-free. Platform bringup (Clearpath drivers, EKF, LiDAR) is
already handled by the systemd ``clearpath-platform`` /
``clearpath-sensors`` services.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare("ridgeback_image_motion")
    default_params = PathJoinSubstitution([pkg, "config", "autonomy_params.yaml"])

    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="r100_0140"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        Node(
            package="ridgeback_image_motion",
            executable="motion_server.py",
            name="motion_server",
            output="screen",
            parameters=[params_file, {"namespace": namespace}],
            emulate_tty=True,
        ),
    ])
