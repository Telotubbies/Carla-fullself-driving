#!/bin/bash
# Kill + restart everything: CARLA (rendered windowed), training, MLflow UI,
# CARLA->ROS2 bridge, RViz2. Intended for one-shot full restart.
set -e
cd "$(dirname "$0")/.."

PW="${PW:-}"
ITERS="${ITERS:-100}"
DISP="${DISPLAY:-:0}"
CARLA_BIN="${CARLA_BIN:-/home/supawich/Desktop/CARLA_0.9.16/CarlaUE4.sh}"
VENV="venv"
ROS_SETUP="/opt/ros/jazzy/setup.bash"

mkdir -p logs

echo "==[1/6]== killing existing processes..."
pkill -9 -f "rviz2" 2>/dev/null || true
pkill -9 -f "carla_rviz_bridge" 2>/dev/null || true
pkill -9 -f "train_rllib_gt" 2>/dev/null || true
pkill -9 -f "ray::" 2>/dev/null || true
pkill -9 -f "CarlaUE4" 2>/dev/null || true
sleep 6
# Clean stale shared memory left by previous CARLA crashes
ipcs -m 2>/dev/null | awk '$6 == "dest" {print $2}' | xargs -r -n1 ipcrm -m >/dev/null 2>&1 || true

echo "==[2/6]== starting CARLA (rendered, windowed)..."
DISPLAY="$DISP" nohup "$CARLA_BIN" -nosound -carla-rpc-port=2000 \
    -quality-level=Low -windowed -ResX=1280 -ResY=720 \
    > logs/carla.log 2>&1 &
disown
CARLA_PID=$!
echo "  CARLA PID=$CARLA_PID"
# Wait for RPC port
for i in 1 2 3 4 5 6; do
    sleep 10
    if ss -ltn 2>/dev/null | grep -q ":2000"; then
        echo "  port 2000 up after ${i}0s"
        break
    fi
    echo "  waiting for CARLA... ${i}0s"
done
"$VENV/bin/python" -c "
import carla
c=carla.Client('localhost',2000); c.set_timeout(30.0)
print('  CARLA ready, map=', c.get_world().get_map().name)
" 2>&1 | tail -2

echo "==[3/6]== verifying MLflow UI..."
if ss -ltn 2>/dev/null | grep -q ":5000"; then
    echo "  MLflow UI already up at http://localhost:5000"
else
    echo "  starting MLflow UI..."
    nohup "$VENV/bin/mlflow" ui --backend-store-uri ./mlruns --host 0.0.0.0 --port 5000 \
        > logs/mlflow.log 2>&1 &
    disown
    sleep 5
fi

echo "==[4/6]== starting training (iters=$ITERS)..."
rm -f logs/training.log
rm -f artifacts/episodes/*.mp4 2>/dev/null || true
nohup "$VENV/bin/python" -u train_rllib_gt.py \
    --iterations "$ITERS" \
    --config config/gt_state.yaml \
    --host localhost --port 2000 \
    > logs/training.log 2>&1 &
disown
TRAIN_PID=$!
echo "  TRAINING PID=$TRAIN_PID"
sleep 3
if ! ps -p "$TRAIN_PID" >/dev/null 2>&1; then
    echo "  training died immediately. Tail of log:" >&2
    tail -20 logs/training.log >&2
    exit 1
fi

echo "==[5/6]== starting CARLA->ROS2 bridge..."
# Wait a bit for training to spawn an ego so the bridge has something to track.
sleep 20
if [ ! -f "$ROS_SETUP" ]; then
    echo "  ROS2 not installed at $ROS_SETUP -- skipping bridge + rviz2"
else
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
    export PYTHONPATH="$(pwd)/venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
    nohup python3 scripts/carla_rviz_bridge.py --host localhost --port 2000 --rate 20 \
        > logs/rviz_bridge.log 2>&1 &
    disown
    BRIDGE_PID=$!
    echo "  BRIDGE PID=$BRIDGE_PID"
    sleep 4
    if ! ps -p "$BRIDGE_PID" >/dev/null 2>&1; then
        echo "  bridge died. Tail:" >&2
        tail -20 logs/rviz_bridge.log >&2
    fi

    echo "==[6/6]== starting RViz2..."
    DISPLAY="$DISP" nohup rviz2 -d "$(pwd)/config/rviz/debug.rviz" \
        > logs/rviz2.log 2>&1 &
    disown
    RVIZ_PID=$!
    echo "  RVIZ2 PID=$RVIZ_PID"
    sleep 4
fi

echo ""
echo "================ STATUS ================"
pgrep -af "CarlaUE4-Linux-Shipping|train_rllib_gt|mlflow ui|carla_rviz_bridge|rviz2" \
    | grep -v "pgrep\|grep\|snapshot" | head -8
echo ""
echo "  MLflow UI : http://localhost:5000"
echo "  CARLA win : on DISPLAY=$DISP (1280x720)"
echo "  RViz2 win : on DISPLAY=$DISP"
echo "  tail -f logs/training.log      # training iter log"
echo "  tail -f logs/rviz_bridge.log   # bridge log"
echo "======================================="
