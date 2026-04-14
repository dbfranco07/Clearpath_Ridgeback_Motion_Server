#!/bin/bash
# Safety Controller Only — Use during initial testing
# Start this before anything else.

set -e

export ROS_DOMAIN_ID=0
export RMW_FASTRTPS_USE_SHM=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/ridgeback/config/fastrtps_jetson.xml

cd ~/ridgeback
source install/setup.bash

echo "Starting Safety Controller + Cmd Vel Mux..."
ros2 launch ridgeback_autonomy safety.launch.py
