#!/bin/bash
# Start RViz2 + CARLA->ROS2 bridge.
#
# Usage:
#   bash scripts/start_rviz.sh           # launches bridge + rviz2 in foreground
#   bash scripts/start_rviz.sh --bg      # runs bridge in background, rviz2 in fg
#
# Expects:
#   - CARLA server running on localhost:2000 with an ego vehicle spawned
#     (the training script provides this).
#   - ROS2 Jazzy installed at /opt/ros/jazzy (apt pkg ros-jazzy-rviz2 etc.)
set -e
cd "$(dirname "$0")/.."

if [ ! -d /opt/ros/jazzy ]; then
    echo "ERROR: /opt/ros/jazzy not found. Install ROS2 Jazzy first." >&2
    exit 1
fi

# Sourcing ROS2 and adding venv site-packages so python3 sees both rclpy and carla.
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

VENV_SITE="$(pwd)/venv/lib/python3.12/site-packages"
export PYTHONPATH="${VENV_SITE}:${PYTHONPATH:-}"

RVIZ_CFG="$(pwd)/config/rviz/debug.rviz"
if [ ! -f "$RVIZ_CFG" ]; then
    echo "ERROR: RViz config not found: $RVIZ_CFG" >&2
    exit 1
fi

mkdir -p logs

echo "[start_rviz] launching carla_rviz_bridge in background..."
pkill -f "scripts/carla_rviz_bridge.py" 2>/dev/null || true
sleep 1
nohup python3 scripts/carla_rviz_bridge.py --host localhost --port 2000 --rate 20 \
    > logs/rviz_bridge.log 2>&1 &
BRIDGE_PID=$!
disown
echo "[start_rviz] bridge PID=$BRIDGE_PID  log=logs/rviz_bridge.log"
sleep 2

if ! ps -p "$BRIDGE_PID" >/dev/null 2>&1; then
    echo "[start_rviz] bridge died. Tail of log:" >&2
    tail -30 logs/rviz_bridge.log >&2
    exit 1
fi

echo "[start_rviz] launching rviz2 with $RVIZ_CFG"
exec rviz2 -d "$RVIZ_CFG"
