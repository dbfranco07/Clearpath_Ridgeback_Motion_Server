#!/bin/bash
# MacBook M4 — Docker Simulation Setup
# =====================================
# Runs the full autonomy stack inside a ROS2 Humble Docker container.
# Includes a mock VLLM server for testing without the real model.

set -e

IMAGE="osrf/ros:humble-desktop"
CONTAINER="ridgeback_sim"
WORKSPACE="$(cd "$(dirname "$0")/../.." && pwd)"

echo "=========================================="
echo "Ridgeback Autonomy — Mac Simulation"
echo "=========================================="
echo "Workspace: $WORKSPACE"

# Check Docker
if ! command -v docker &> /dev/null; then
  echo "ERROR: Docker not installed. Install from https://www.docker.com/"
  exit 1
fi

# Pull ROS2 image if needed
echo "[1/4] Pulling ROS2 Humble image (if needed)..."
docker pull $IMAGE

# Build custom image with our dependencies
echo "[2/4] Building simulation image..."
cat > /tmp/ridgeback_dockerfile << 'EOF'
FROM osrf/ros:humble-desktop

# Install Nav2, SLAM Toolbox, TurtleBot3
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-turtlebot3-gazebo \
    ros-humble-gazebo-ros-pkgs \
    python3-pip \
    python3-opencv \
    sqlite3 \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install fastapi uvicorn pydantic aiohttp

# Set up workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws
EOF

docker build -t ridgeback_sim_image -f /tmp/ridgeback_dockerfile .

# Start mock VLLM server (Python HTTP server returning fake VLM responses)
echo "[3/4] Starting mock VLLM server..."
python3 - << 'PYEOF' &
MOCK_PID=$!
from http.server import HTTPServer, BaseHTTPRequestHandler
import json, random

RESPONSES = [
    "ENVIRONMENT: hallway\nROOM_NUMBERS: NONE\nOBSTACLES: NO\nDESCRIPTION: Empty corridor with fluorescent lighting.",
    "ENVIRONMENT: hallway\nROOM_NUMBERS: 301\nOBSTACLES: NO\nDESCRIPTION: Door on the right with room number 301 visible.",
    "ENVIRONMENT: hallway\nROOM_NUMBERS: 303\nOBSTACLES: YES\nDESCRIPTION: Hallway with a chair near room 303.",
    "ENVIRONMENT: hallway\nROOM_NUMBERS: 305\nOBSTACLES: NO\nDESCRIPTION: End of corridor, room 305 on the left.",
    "ENVIRONMENT: intersection\nROOM_NUMBERS: NONE\nOBSTACLES: NO\nDESCRIPTION: T-intersection ahead.",
]

class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        length = int(self.headers.get('Content-Length', 0))
        self.rfile.read(length)
        response = {"choices": [{"message": {"content": random.choice(RESPONSES)}}]}
        body = json.dumps(response).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', len(body))
        self.end_headers()
        self.wfile.write(body)
    def log_message(self, *args): pass

print("Mock VLLM server on port 9999")
HTTPServer(('0.0.0.0', 9999), Handler).serve_forever()
PYEOF

# Start simulation container
echo "[4/4] Starting simulation container..."
echo "=========================================="
echo "Dashboard: http://localhost:8081"
echo "Press Ctrl+C to stop"
echo "=========================================="

docker run -it --rm \
  --name $CONTAINER \
  -e ROS_DOMAIN_ID=0 \
  -e TURTLEBOT3_MODEL=burger \
  -p 8081:8081 \
  -p 9999:9999 \
  -v "$WORKSPACE:/ros2_ws/src/ridgeback" \
  --network host \
  $IMAGE \
  bash -c "
    source /opt/ros/humble/setup.bash && \
    cd /ros2_ws && \
    colcon build --packages-select ridgeback_image_motion ridgeback_autonomy && \
    source install/setup.bash && \
    ros2 launch ridgeback_autonomy simulation.launch.py
  "

# Cleanup mock server
kill $MOCK_PID 2>/dev/null || true
