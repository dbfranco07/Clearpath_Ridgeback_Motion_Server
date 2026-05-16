"""Bring up the Intel RealSense D435 directly on the Jetson.

The camera body is still mounted on the Ridgeback (parent: top_link in
robot.yaml). Only the USB cable is plugged into the Jetson now, so the
Ridgeback's `clearpath-sensors.service` must have `launch_enabled: false`
for the camera (URDF stays enabled so the static TF tree to the
camera_0_*_optical_frame frames is still emitted by the Ridgeback's
robot_state_publisher).

We deliberately disable the driver's own TF publication because the
URDF on the Ridgeback already owns those static transforms.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    serial_no = LaunchConfiguration("serial_no")
    camera_name = LaunchConfiguration("camera_name")
    camera_namespace = LaunchConfiguration("camera_namespace")

    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name=camera_name,
        namespace=camera_namespace,
        output="screen",
        parameters=[{
            # realsense2_camera_node declares serial_no as a string; without
            # the explicit ParameterValue wrap, all-numeric serials get coerced
            # to int and the node refuses to start.
            "serial_no": ParameterValue(serial_no, value_type=str),
            "camera_name": camera_name,
            # USB 2 mode: RGB only at 640x480x6. Depth + sync + aligned-depth
            # all require USB 3 bandwidth. Re-enable once on a USB 3 cable/port.
            "enable_color": True,
            "enable_depth": False,
            "enable_sync": False,
            "align_depth.enable": False,
            "rgb_camera.color_profile": "640x480x6",
            "enable_infra1": False,
            "enable_infra2": False,
            "enable_gyro": False,
            "enable_accel": False,
            "publish_tf": False,
            "pointcloud.enable": False,
            "decimation_filter.enable": False,
            "spatial_filter.enable": False,
            "temporal_filter.enable": False,
            "hole_filling_filter.enable": False,
            "json_file_path": "",
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_no",
            default_value="317222071726",
            description="RealSense device serial. Matches the D435 listed in CLAUDE.md.",
        ),
        DeclareLaunchArgument(
            "camera_name",
            default_value="camera_0",
            description="Must match the camera frame_id prefix used by the Ridgeback URDF.",
        ),
        DeclareLaunchArgument(
            "camera_namespace",
            default_value="r100_0140/sensors",
            description="Namespace so topics land under /r100_0140/sensors/camera_0/*.",
        ),
        realsense_node,
    ])
