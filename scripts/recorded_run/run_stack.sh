#!/usr/bin/env bash
# run_stack.sh - launch the recorded validation stack in its OWN process group with a
# PID manifest, so shutdown only ever touches processes this run started. Never kills
# other ROS sessions, never touches global /dev/shm DDS artifacts. Isolation comes from
# ROS_DOMAIN_ID + GZ_PARTITION (already per-run) plus this process-group scoping.
#
# Usage: run_stack.sh <run_dir> <ros2 launch args...>
set -euo pipefail
RUN_DIR="$1"; shift
mkdir -p "$RUN_DIR"

# --- PRE-FLIGHT (fail closed) -------------------------------------------------
if pgrep -f "gz sim" >/dev/null; then
  echo "PREFLIGHT FAIL: a gz sim process is already running. This script does not kill" >&2
  echo "processes it did not start. Stop the other session yourself, then re-run." >&2
  exit 1
fi
CLOCK_PUBS=$(timeout 5 ros2 topic info /clock 2>/dev/null | grep -c "Node:" || true)
if [ "${CLOCK_PUBS:-0}" != "0" ]; then
  echo "PREFLIGHT FAIL: /clock already has $CLOCK_PUBS publisher(s) in this ROS domain -" >&2
  echo "another sim is active here. Choose another ROS_DOMAIN_ID or stop it yourself." >&2
  exit 1
fi

# --- LAUNCH in a new process group --------------------------------------------
setsid ros2 launch "$@" >"$RUN_DIR/launch.out" 2>&1 &
LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$RUN_DIR/stack.pgid"   # PGID == PID of the setsid leader
ps -o cmd= -p "$LAUNCH_PID" > "$RUN_DIR/stack.cmd" || true
echo "stack launched: PGID $LAUNCH_PID (manifest in $RUN_DIR)"
