#!/bin/bash
# Cleanly restart the GT SAC training. Run from repo root.
set -e
cd "$(dirname "$0")/.."

echo "[restart] killing any existing training + Ray workers..."
pkill -9 -f "train_rllib_gt" 2>/dev/null || true
pkill -9 -f "ray::" 2>/dev/null || true
sleep 6

echo "[restart] cleaning logs + artifacts..."
rm -f logs/training.log
rm -f artifacts/episodes/*.mp4 2>/dev/null || true

ITERS="${ITERS:-30}"
echo "[restart] launching training (iters=$ITERS)..."
nohup venv/bin/python -u train_rllib_gt.py \
    --iterations "$ITERS" \
    --config config/gt_state.yaml \
    --host localhost --port 2000 \
    > logs/training.log 2>&1 &
PID=$!
disown
echo "[restart] PID=$PID"
sleep 3
if ps -p "$PID" >/dev/null 2>&1; then
    echo "[restart] training is running."
else
    echo "[restart] training died immediately. Tail of log:" >&2
    tail -30 logs/training.log >&2
    exit 1
fi
