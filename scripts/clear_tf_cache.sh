#!/usr/bin/env bash
set -euo pipefail

# Purpose:
# - Avoid stale TF/clock state from previous runs (especially with /use_sim_time)
# - This is the most reliable "clear TF cache" in ROS1: restart the processes
#   that hold TF buffers (rviz, move_base, amcl, etc.) and (optionally) roscore/gazebo.
#
# This script is intentionally conservative by default:
# - It only kills common TF-buffered nodes if a ROS master is reachable.
# - It does NOT kill rosmaster/roscore unless you export FORCE_KILL_ROSCORE=1.

FORCE_KILL_ROSCORE="${FORCE_KILL_ROSCORE:-0}"
FORCE_KILL_GAZEBO="${FORCE_KILL_GAZEBO:-0}"

_have_rosmaster() {
  command -v rosnode >/dev/null 2>&1 && rosnode list >/dev/null 2>&1
}

_kill_node_if_exists() {
  local node="$1"
  rosnode info "$node" >/dev/null 2>&1 || return 0
  echo "[clear_tf_cache] rosnode kill ${node}"
  rosnode kill "$node" >/dev/null 2>&1 || true
}

main() {
  if _have_rosmaster; then
    # Kill typical TF listeners/buffer holders first.
    _kill_node_if_exists "/rviz" || true
    _kill_node_if_exists "/move_base" || true
    _kill_node_if_exists "/amcl" || true

    # Some runs may namespace nodes; as a best-effort, kill all rviz instances registered in ROS.
    # (If there are no matches, xargs does nothing.)
    rosnode list | grep -E '(^|/)rviz$' | xargs -r -n1 rosnode kill >/dev/null 2>&1 || true
  fi

  # Best-effort kill of leftover GUI processes (sometimes not registered as ROS nodes).
  pkill -SIGINT -f "\brviz\b" >/dev/null 2>&1 || true

  if [[ "$FORCE_KILL_GAZEBO" == "1" ]]; then
    echo "[clear_tf_cache] FORCE_KILL_GAZEBO=1, killing gzserver/gzclient"
    pkill -SIGINT -f "\bgzserver\b" >/dev/null 2>&1 || true
    pkill -SIGINT -f "\bgzclient\b" >/dev/null 2>&1 || true
  fi

  if [[ "$FORCE_KILL_ROSCORE" == "1" ]]; then
    echo "[clear_tf_cache] FORCE_KILL_ROSCORE=1, killing rosmaster/roscore/gazebo"
    pkill -9 -f "\brosmaster\b" >/dev/null 2>&1 || true
    pkill -9 -f "\broscore\b" >/dev/null 2>&1 || true
    pkill -SIGINT -f "\bgzserver\b" >/dev/null 2>&1 || true
    pkill -SIGINT -f "\bgzclient\b" >/dev/null 2>&1 || true
  fi

  # Give processes a moment to exit so new nodes start cleanly.
  sleep 0.5
}

main "$@"
