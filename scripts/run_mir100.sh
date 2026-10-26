#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
if [[ ! -f "$ROS_SETUP" || ! -f "$WORKSPACE_DIR/install/local_setup.bash" ]]; then
  echo "Build this workspace first; see README.md. ROS setup: $ROS_SETUP" >&2
  exit 1
fi
set +u
source "$ROS_SETUP"
source "$WORKSPACE_DIR/install/local_setup.bash"
set -u
if ! ros2 launch --help >/dev/null 2>&1; then
  echo "ROS launch CLI is unavailable. Install ros-jazzy-ros2launch or set ROS_SETUP to your complete ROS setup.bash." >&2
  exit 1
fi
# This simulation runs entirely on one laptop, independent of Wi-Fi changes.
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_IP="${GZ_IP:-127.0.0.1}"

if command -v systemd-inhibit >/dev/null 2>&1; then
  # Released automatically when the launch exits; no power settings are changed.
  exec systemd-inhibit --what=sleep:idle --mode=block --who="MiR100 simulation" \
    --why="Keep Gazebo and Nav2 clocks and heartbeats running" \
    ros2 launch mir100_navigation slam_navigation.launch.py "$@"
fi
echo "systemd-inhibit unavailable: keep the computer awake during the simulation." >&2
exec ros2 launch mir100_navigation slam_navigation.launch.py "$@"
