#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/autoracer/unilidar_sdk/unitree_lidar_ros2/install/setup.bash
source /home/autoracer/f1tenth_ws/install/setup.bash
set -u
export ROS_DOMAIN_ID=0
export PYTHONDONTWRITEBYTECODE=1

cd "$(dirname "${BASH_SOURCE[0]}")"

# Deliberately dry-run by default. Add --drive only after replay/stationary
# validation with the real IROS scans and a verified lap length/offset.
exec python3 -u track_wall_explorer_node.py "$@"
