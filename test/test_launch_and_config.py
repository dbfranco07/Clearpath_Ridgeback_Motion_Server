from __future__ import annotations

from conftest import read_repo_file


def test_autonomy_launch_contains_jetson_owned_nodes_and_optional_vslam() -> None:
    launch = read_repo_file("ridgeback_image_motion", "launch", "autonomy.launch.py")
    for token in (
        "safety_controller.py",
        "jetson_watchdog.py",
        "cmd_vel_mux.py",
        "mission_orchestrator.py",
        "frontier_explorer.py",
        "room_detector.py",
        "web_dashboard.py",
        "rtabmap_odom",
        "robot_localization",
        "launch_vslam",
    ):
        assert token in launch
    assert 'default_value="false"' in launch


def test_nav2_and_slam_frame_topic_contracts_match_ridgeback() -> None:
    nav2 = read_repo_file("config", "nav2_params.yaml")
    slam = read_repo_file("config", "slam_params.yaml")
    for text in (nav2, slam):
        assert "base_link" in text
        assert "/r100_0140/sensors/lidar2d_0/scan" in text
    assert "global_frame: map" in nav2
    assert "odom_frame: odom" in slam
    assert "base_frame: base_link" in slam
    assert "scan_topic: /r100_0140/sensors/lidar2d_0/scan" in slam


def test_ridgeback_web_script_launches_autonomy_stack() -> None:
    script = read_repo_file("scripts", "ridgeback_web.sh")
    assert "ros2 launch ridgeback_image_motion autonomy.launch.py" in script
    assert 'RIDGEBACK_PROFILE="${RIDGEBACK_PROFILE:-mission}"' in script
    assert 'profile:="$RIDGEBACK_PROFILE"' in script


def test_ridgeback_web_script_supports_nav2_off_mode() -> None:
    script = read_repo_file("scripts", "ridgeback_web.sh")
    assert "--no-nav2" in script
    assert "RIDGEBACK_LAUNCH_NAV2=false" in script
    assert 'RIDGEBACK_LAUNCH_NAV2="${RIDGEBACK_LAUNCH_NAV2:-auto}"' in script
    assert 'launch_nav2:="$RIDGEBACK_LAUNCH_NAV2"' in script


def test_ridgeback_web_script_exposes_effective_launch_settings() -> None:
    script = read_repo_file("scripts", "ridgeback_web.sh")
    for setting in (
        "RIDGEBACK_LAUNCH_SLAM",
        "RIDGEBACK_LAUNCH_NAV2",
        "RIDGEBACK_LAUNCH_VLM",
        "RIDGEBACK_LAUNCH_DASHBOARD",
        "RIDGEBACK_LAUNCH_VSLAM",
    ):
        assert setting in script
        assert f"echo \"  {setting}=$" in script


def test_protected_ridgeback_start_contract_remains_minimal() -> None:
    script = read_repo_file("scripts", "ridgeback_start.sh")
    assert "motion_server.py" in script
    assert "image_publisher.py" in script
    assert "autonomy.launch.py" not in script
    assert "nav2" not in script.lower()
    assert "slam" not in script.lower()


def test_dashboard_camera_fallbacks_are_available_in_mission_profile() -> None:
    launch = read_repo_file("ridgeback_image_motion", "launch", "autonomy.launch.py")
    dashboard = read_repo_file("ridgeback_image_motion", "web_dashboard.py")
    params = read_repo_file("config", "autonomy_params.yaml")

    assert "auto_raw_camera_fallback" in launch
    assert "' in ['mission', 'debug']" in launch
    assert "fallback_compressed_image_topic" in dashboard
    assert "/r100_0140/sensors/camera_0/color/compressed" in params
    assert "/r100_0140/sensors/camera_0/color/image" in params
    assert "/r100_0140/sensors/camera_0/depth/image" in params


def test_dashboard_debug_reports_camera_fallback_state() -> None:
    dashboard = read_repo_file("ridgeback_image_motion", "web_dashboard.py")
    for token in (
        "camera_stale_reason",
        "depth_stale_reason",
        "camera_topic",
        "depth_topic",
        "image_fallback",
        "compressed_fallback",
        "raw_fallback",
    ):
        assert token in dashboard


def test_vslam_config_is_present_but_not_default_enabled() -> None:
    launch = read_repo_file("ridgeback_image_motion", "launch", "autonomy.launch.py")
    vslam = read_repo_file("config", "vslam_params.yaml")
    assert "launch_vslam" in launch
    assert '"launch_vslam",' in launch
    assert "odom0: /r100_0140/platform/odom/filtered" in vslam
    assert "odom1: /ridgeback/vslam/odom" in vslam
