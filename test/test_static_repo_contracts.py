from __future__ import annotations

from conftest import REPO_ROOT, read_repo_file


def tracked_python_and_config_files() -> list:
    patterns = ("*.py", "*.yaml", "*.yml", "*.xml", "*.sh", "*.md")
    files = []
    for pattern in patterns:
        files.extend(REPO_ROOT.glob(f"**/{pattern}"))
    return [
        path
        for path in files
        if "ridgeback_autonomy" not in path.parts
        and "__pycache__" not in path.parts
        and ".git" not in path.parts
    ]


def test_no_code_imports_or_depends_on_disregarded_ridgeback_autonomy() -> None:
    offenders = []
    for path in tracked_python_and_config_files():
        if path.suffix == ".md":
            continue
        text = path.read_text(errors="ignore")
        if "ridgeback_autonomy" in text and "test_static_repo_contracts.py" not in str(path):
            offenders.append(str(path.relative_to(REPO_ROOT)))
    assert offenders == []


def test_no_active_preloaded_map_references() -> None:
    offenders = []
    for path in tracked_python_and_config_files():
        for line in path.read_text(errors="ignore").splitlines():
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            if stripped.startswith("map_file_name:") or stripped.startswith("serialized_map_file_name:"):
                offenders.append(f"{path.relative_to(REPO_ROOT)}: {stripped}")
    assert offenders == []


def test_protected_ridgeback_side_files_are_not_autonomy_entrypoints() -> None:
    start = read_repo_file("scripts", "ridgeback_start.sh")
    image_pub = read_repo_file("ridgeback_image_motion", "image_publisher.py")
    motion_server = read_repo_file("ridgeback_image_motion", "motion_server.py")
    assert "autonomy.launch.py" not in start
    assert "nav2" not in image_pub.lower()
    assert "slam" not in image_pub.lower()
    assert "nav2" not in motion_server.lower()
    assert "slam" not in motion_server.lower()


def test_sim_launch_is_no_longer_stale_todo_wrapper() -> None:
    sim_launch = read_repo_file("sim", "launch", "sim.launch.py")
    assert "autonomy.launch.py" in sim_launch
    assert "TODO: uncomment" not in sim_launch
    assert "mission_orchestrator.py" not in sim_launch


def test_scripts_document_jetson_and_ridgeback_split() -> None:
    web = read_repo_file("scripts", "ridgeback_web.sh")
    start = read_repo_file("scripts", "ridgeback_start.sh")
    assert "Starting Jetson autonomy stack" in web
    assert "Starting motion server" in start
    assert "Starting image publisher" in start
