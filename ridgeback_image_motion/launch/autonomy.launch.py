from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port")
    host = LaunchConfiguration("host")

    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="0.0.0.0"),
        DeclareLaunchArgument("port", default_value="8081"),
        Node(
            package="ridgeback_image_motion",
            executable="web_dashboard.py",
            name="ridgeback_dashboard",
            output="screen",
            parameters=[{"host": host, "port": port}],
        ),
    ])