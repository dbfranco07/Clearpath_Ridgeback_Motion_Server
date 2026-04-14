"""
SLAM + Nav2 Launch
==================
Starts SLAM Toolbox (online async) and the Nav2 stack.
Requires the Ridgeback's clearpath-robot.service to be running first.
Also requires safety.launch.py to be running (or use autonomy.launch.py).

Topic remappings:
  - SLAM Toolbox and Nav2 expect standard topic names
  - We remap Ridgeback's namespaced topics here
  - Nav2's cmd_vel output goes to /cmd_vel_nav2 (picked up by mux)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('ridgeback_autonomy')
    nav2_pkg = FindPackageShare('nav2_bringup')

    slam_params = PathJoinSubstitution([pkg, 'config', 'slam_toolbox_params.yaml'])
    nav2_params = PathJoinSubstitution([pkg, 'config', 'nav2_params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('slam_params', default_value=slam_params),
        DeclareLaunchArgument('nav2_params', default_value=nav2_params),

        # Static TF: map → odom (initial transform, SLAM will update this)
        # Note: SLAM Toolbox manages the map→odom transform itself once running

        # SLAM Toolbox (online async mode — builds map while navigating)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                LaunchConfiguration('slam_params'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                # SLAM Toolbox reads from standard /scan, remap to Ridgeback's
                ('/scan', '/r100_0140/sensors/lidar2d_0/scan'),
                ('/odom', '/r100_0140/platform/odom/filtered'),
                ('/tf', '/r100_0140/tf'),
                ('/tf_static', '/r100_0140/tf_static'),
            ],
            output='screen',
            emulate_tty=True
        ),

        # Nav2 Bringup (navigation stack — uses /map from SLAM Toolbox)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                nav2_pkg, '/launch/navigation_launch.py'
            ]),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('nav2_params'),
                # Nav2 publishes cmd_vel to this topic — cmd_vel_mux picks it up
                'cmd_vel_topic': '/cmd_vel_nav2',
            }.items()
        ),
    ])
