"""
Safety Controller Launch — Standalone
======================================
Starts ONLY the safety controller and cmd_vel_mux.
Use this to run safety monitoring without the full autonomy stack.
Always run this first.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('ridgeback_autonomy')
    params_file = PathJoinSubstitution([pkg, 'config', 'safety_params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Path to safety parameters YAML'
        ),

        # Safety Controller (highest priority — start first)
        Node(
            package='ridgeback_autonomy',
            executable='safety_controller.py',
            name='safety_controller',
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
            emulate_tty=True
        ),

        # Cmd Vel Mux — arbitrates between safety / nav2 / teleop
        Node(
            package='ridgeback_autonomy',
            executable='cmd_vel_mux.py',
            name='cmd_vel_mux',
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
            emulate_tty=True
        ),
    ])
