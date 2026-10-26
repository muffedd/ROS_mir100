#!/usr/bin/env bash
# stop_stack.sh - graceful, MANIFEST-SCOPED shutdown of a stack started by run_stack.sh.
# Verifies ownership before signalling: only the recorded process group, only if its
# leader still matches the recorded launch command. No broad pkill, no shared-state
# deletion. Other ROS sessions and their /dev/shm artifacts are never touched.
set -euo pipefail
RUN_DIR="$1"
[ -f "$RUN_DIR/stack.pgid" ] || { echo "no manifest in $RUN_DIR - nothing to stop"; exit 0; }
PGID=$(cat "$RUN_DIR/stack.pgid")
WANT_CMD=$(cat "$RUN_DIR/stack.cmd" 2>/dev/null || echo "")
if [ ! -d "/proc/$PGID" ]; then
  echo "process group $PGID already gone"; exit 0
fi
NOW_CMD=$(ps -o cmd= -p "$PGID" || true)
if [ -n "$WANT_CMD" ] && [ "$NOW_CMD" != "$WANT_CMD" ]; then
  echo "OWNERSHIP MISMATCH: PID $PGID now runs '$NOW_CMD', expected '$WANT_CMD'." >&2
  echo "Refusing to signal a reused PID. Inspect manually." >&2
  exit 1
fi
kill -TERM -- -"$PGID" 2>/dev/null || true   # graceful: whole group, TERM first
for i in $(seq 1 15); do [ -d "/proc/$PGID" ] || break; sleep 1; done
if [ -d "/proc/$PGID" ]; then
  NOW_CMD2=$(ps -o cmd= -p "$PGID" || true)
  if [ "$NOW_CMD2" = "$NOW_CMD" ]; then kill -KILL -- -"$PGID" 2>/dev/null || true; fi
fi
sleep 1
[ ! -d "/proc/$PGID" ] && echo "stack $PGID stopped cleanly" || { echo "WARNING: $PGID still present"; exit 1; }
