#!/usr/bin/env bash

set -euo pipefail

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_DIR="$(cd "$PACKAGE_DIR/../.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS setup file not found: $ROS_SETUP" >&2
  exit 1
fi

set +u
source "$ROS_SETUP"
source "$WORKSPACE_DIR/install/setup.bash"
set -u

MAP_DIR="$PACKAGE_DIR/maps"
mkdir -p "$MAP_DIR"
ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/mir100_world"

echo "Saved map files under $MAP_DIR"
