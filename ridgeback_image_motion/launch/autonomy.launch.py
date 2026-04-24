from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    launch_slam = LaunchConfiguration("launch_slam")
    launch_nav2 = LaunchConfiguration("launch_nav2")
    launch_vlm = LaunchConfiguration("launch_vlm")
    launch_dashboard = LaunchConfiguration("launch_dashboard")

    pkg_share = FindPackageShare("ridgeback_image_motion")
    autonomy_params = PathJoinSubstitution([pkg_share, "config", "autonomy_params.yaml"])
    slam_params = PathJoinSubstitution([pkg_share, "config", "slam_params.yaml"])
    nav2_params = PathJoinSubstitution([pkg_share, "config", "nav2_params.yaml"])

    nav2_launch = PathJoinSubstitution(
        [get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py"]
    )

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params],
        condition=IfCondition(launch_slam),
    )

    nav2 = GroupAction(
        condition=IfCondition(launch_nav2),
        actions=[
            SetRemap(src="/cmd_vel", dst="/cmd_vel_nav_raw"),
            SetRemap(src="cmd_vel", dst="/cmd_vel_nav_raw"),
            SetRemap(src="/cmd_vel_smoothed", dst="/cmd_vel_nav"),
            SetRemap(src="cmd_vel_smoothed", dst="/cmd_vel_nav"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "use_sim_time": "false",
                    "autostart": "true",
                    "params_file": nav2_params,
                }.items(),
            ),
        ],
    )

    safety = Node(
        package="ridgeback_image_motion",
        executable="safety_controller.py",
        name="safety_controller",
        output="screen",
        parameters=[autonomy_params],
    )

    mux = Node(
        package="ridgeback_image_motion",
        executable="cmd_vel_mux.py",
        name="cmd_vel_mux",
        output="screen",
        parameters=[autonomy_params],
    )

    mission = Node(
        package="ridgeback_image_motion",
        executable="mission_orchestrator.py",
        name="mission_orchestrator",
        output="screen",
        parameters=[autonomy_params],
    )

    frontier = Node(
        package="ridgeback_image_motion",
        executable="frontier_explorer.py",
        name="frontier_explorer",
        output="screen",
        parameters=[autonomy_params],
    )

    room_detector = Node(
        package="ridgeback_image_motion",
        executable="room_detector.py",
        name="room_detector",
        output="screen",
        parameters=[autonomy_params],
        condition=IfCondition(launch_vlm),
    )

    dashboard = Node(
        package="ridgeback_image_motion",
        executable="web_dashboard.py",
        name="ridgeback_dashboard",
        output="screen",
        parameters=[autonomy_params, {"host": host, "port": port}],
        condition=IfCondition(launch_dashboard),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8081"),
            DeclareLaunchArgument("launch_slam", default_value="true"),
            DeclareLaunchArgument("launch_nav2", default_value="true"),
            DeclareLaunchArgument("launch_vlm", default_value="true"),
            DeclareLaunchArgument("launch_dashboard", default_value="true"),
            LogInfo(msg="Starting Ridgeback Jetson autonomy stack: safety, mux, SLAM, Nav2, exploration, VLM, dashboard"),
            safety,
            mux,
            TimerAction(period=1.0, actions=[slam]),
            TimerAction(period=3.0, actions=[nav2]),
            TimerAction(period=6.0, actions=[mission, frontier, room_detector]),
            TimerAction(period=8.0, actions=[dashboard]),
        ]
    )
