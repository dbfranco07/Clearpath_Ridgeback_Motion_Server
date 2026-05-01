from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def profile_condition(profile, override, enabled_profiles):
    profiles = repr(list(enabled_profiles))
    return IfCondition(
        PythonExpression([
            "'", override, "' == 'true' or ('", override, "' == 'auto' and '", profile, "' in ", profiles, ")"
        ])
    )


def generate_launch_description():
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    profile = LaunchConfiguration("profile")
    launch_slam = LaunchConfiguration("launch_slam")
    launch_nav2 = LaunchConfiguration("launch_nav2")
    launch_vlm = LaunchConfiguration("launch_vlm")
    launch_vslam = LaunchConfiguration("launch_vslam")
    launch_dashboard = LaunchConfiguration("launch_dashboard")
    vslam_rgb_topic = LaunchConfiguration("vslam_rgb_topic")
    vslam_depth_topic = LaunchConfiguration("vslam_depth_topic")
    vslam_camera_info_topic = LaunchConfiguration("vslam_camera_info_topic")
    vslam_output_odom_topic = LaunchConfiguration("vslam_output_odom_topic")

    slam_condition = profile_condition(profile, launch_slam, ["mapping", "mission", "debug"])
    nav2_condition = profile_condition(profile, launch_nav2, ["mission", "debug"])
    vlm_condition = profile_condition(profile, launch_vlm, ["mission", "debug"])
    vslam_condition = IfCondition(PythonExpression(["'", launch_vslam, "' == 'true'"]))
    dashboard_condition = profile_condition(profile, launch_dashboard, ["teleop", "mapping", "mission", "debug"])
    mission_condition = IfCondition(PythonExpression(["'", profile, "' in ['mission', 'debug']"]))

    pkg_share = FindPackageShare("ridgeback_image_motion")
    autonomy_params = PathJoinSubstitution([pkg_share, "config", "autonomy_params.yaml"])
    slam_params = PathJoinSubstitution([pkg_share, "config", "slam_params.yaml"])
    nav2_params = PathJoinSubstitution([pkg_share, "config", "nav2_params.yaml"])
    vslam_params = PathJoinSubstitution([pkg_share, "config", "vslam_params.yaml"])

    nav2_launch = PathJoinSubstitution(
        [get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py"]
    )

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params],
        remappings=[
            ("/tf", "/r100_0140/tf"),
            ("/tf_static", "/r100_0140/tf_static"),
        ],
        condition=slam_condition,
    )

    nav2 = GroupAction(
        condition=nav2_condition,
        actions=[
            SetRemap(src="/cmd_vel", dst="/cmd_vel_nav_raw"),
            SetRemap(src="cmd_vel", dst="/cmd_vel_nav_raw"),
            SetRemap(src="/cmd_vel_smoothed", dst="/cmd_vel_nav"),
            SetRemap(src="cmd_vel_smoothed", dst="/cmd_vel_nav"),
            SetRemap(src="/tf", dst="/r100_0140/tf"),
            SetRemap(src="/tf_static", dst="/r100_0140/tf_static"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "namespace": "",
                    "use_sim_time": "false",
                    "use_composition": "False",
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

    watchdog = Node(
        package="ridgeback_image_motion",
        executable="jetson_watchdog.py",
        name="jetson_watchdog",
        output="screen",
        parameters=[{
            "heartbeat_timeout": 4.0,
            "init_grace_period": 20.0,
            "cmd_vel_topic": "/cmd_vel_safety",
            "require_initial_heartbeat": True,
        }],
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
        remappings=[
            ("/tf", "/r100_0140/tf"),
            ("/tf_static", "/r100_0140/tf_static"),
        ],
        condition=mission_condition,
    )

    frontier = Node(
        package="ridgeback_image_motion",
        executable="frontier_explorer.py",
        name="frontier_explorer",
        output="screen",
        parameters=[autonomy_params],
        remappings=[
            ("/tf", "/r100_0140/tf"),
            ("/tf_static", "/r100_0140/tf_static"),
        ],
        condition=mission_condition,
    )

    room_detector = Node(
        package="ridgeback_image_motion",
        executable="room_detector.py",
        name="room_detector",
        output="screen",
        parameters=[autonomy_params],
        remappings=[
            ("/tf", "/r100_0140/tf"),
            ("/tf_static", "/r100_0140/tf_static"),
        ],
        condition=vlm_condition,
    )

    rgbd_odometry = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rtabmap_rgbd_odometry",
        output="screen",
        parameters=[{
            "frame_id": "base_link",
            "odom_frame_id": "rtabmap_odom",
            "publish_tf": False,
            "approx_sync": True,
            "approx_sync_max_interval": 0.08,
            "queue_size": 10,
        }],
        remappings=[
            ("rgb/image", vslam_rgb_topic),
            ("depth/image", vslam_depth_topic),
            ("rgb/camera_info", vslam_camera_info_topic),
            ("odom", "/ridgeback/vslam/odom"),
        ],
        condition=vslam_condition,
    )

    vslam_fusion = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ridgeback_vslam_ekf",
        output="screen",
        parameters=[vslam_params],
        remappings=[
            ("odometry/filtered", vslam_output_odom_topic),
        ],
        condition=vslam_condition,
    )

    dashboard = Node(
        package="ridgeback_image_motion",
        executable="web_dashboard.py",
        name="ridgeback_dashboard",
        output="screen",
        parameters=[
            autonomy_params,
            {
                "host": host,
                "port": port,
                "raw_image_topic": PythonExpression([
                    "'' if '", profile, "' == 'teleop' else '/r100_0140/sensors/camera_0/color/image'"
                ]),
                "depth_topic": PythonExpression([
                    "'/r100_0140/sensors/camera_0/depth/image' if '", profile, "' == 'debug' else ''"
                ]),
                "auto_raw_camera_fallback": True,
                "raw_fallback_after_s": 8.0,
                "enable_depth_feed": ParameterValue(
                    PythonExpression(["'", profile, "' in ['mission', 'debug']"]),
                    value_type=bool,
                ),
                "rgb_stream_hz": PythonExpression([
                    "12.0 if '", profile, "' == 'debug' else 8.0"
                ]),
                "depth_render_hz": PythonExpression([
                    "4.0 if '", profile, "' == 'debug' else 1.0"
                ]),
            },
        ],
        condition=dashboard_condition,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8081"),
            DeclareLaunchArgument(
                "profile",
                default_value="mission",
                description="Runtime profile: teleop, mapping, mission, or debug",
            ),
            DeclareLaunchArgument("launch_slam", default_value="auto"),
            DeclareLaunchArgument("launch_nav2", default_value="auto"),
            DeclareLaunchArgument("launch_vlm", default_value="auto"),
            DeclareLaunchArgument(
                "launch_vslam",
                default_value="false",
                description="Optional RTAB-Map RGB-D odometry + EKF fusion path. Disabled until Jetson load/TF validation.",
            ),
            DeclareLaunchArgument(
                "vslam_rgb_topic",
                default_value="/r100_0140/sensors/camera_0/color/image",
                description="RGB image topic for RTAB-Map RGB-D odometry.",
            ),
            DeclareLaunchArgument(
                "vslam_depth_topic",
                default_value="/r100_0140/sensors/camera_0/depth/image",
                description="Depth image topic for RTAB-Map. Prefer an aligned-depth-to-color topic if available.",
            ),
            DeclareLaunchArgument(
                "vslam_camera_info_topic",
                default_value="/r100_0140/sensors/camera_0/color/camera_info",
                description="CameraInfo topic matching the RGB image used by vSLAM.",
            ),
            DeclareLaunchArgument(
                "vslam_output_odom_topic",
                default_value="/ridgeback/vslam/odom_filtered",
                description="Filtered odometry output from the optional vSLAM EKF.",
            ),
            DeclareLaunchArgument("launch_dashboard", default_value="auto"),
            LogInfo(msg=["Starting Ridgeback Jetson autonomy stack profile=", profile]),
            LogInfo(
                msg="Profiles: teleop=safety/mux/dashboard, mapping=+SLAM, mission=+Nav2/exploration/VLM, debug=full stack with raw fallbacks"
            ),
            safety,
            watchdog,
            mux,
            TimerAction(period=1.0, actions=[slam]),
            TimerAction(period=8.0, actions=[nav2]),
            TimerAction(period=10.0, actions=[dashboard]),
            TimerAction(period=12.0, actions=[rgbd_odometry, vslam_fusion]),
            TimerAction(period=18.0, actions=[mission, frontier, room_detector]),
        ]
    )
