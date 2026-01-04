#!/bin/bash
# Start TensorBoard Server for Dashboard Integration

set -e

RL_AGENT_DIR="/home/a/Desktop/CARLA_0.9.16/RL_Agent"
TENSORBOARD_PORT=6006
TENSORBOARD_LOG="/tmp/tensorboard.log"

cd "$RL_AGENT_DIR"

# Check if TensorBoard is already running
if ps aux | grep -E "tensorboard.*--port=$TENSORBOARD_PORT" | grep -v grep > /dev/null; then
    echo "✅ TensorBoard is already running on port $TENSORBOARD_PORT"
    exit 0
fi

# Check if port is in use
if netstat -tlnp 2>/dev/null | grep ":$TENSORBOARD_PORT " > /dev/null || \
   ss -tlnp 2>/dev/null | grep ":$TENSORBOARD_PORT " > /dev/null; then
    echo "⚠️  Port $TENSORBOARD_PORT is already in use"
    exit 1
fi

echo "🚀 Starting TensorBoard server..."
echo "   Logdir: logs/tensorboard"
echo "   Port: $TENSORBOARD_PORT"
echo "   Log: $TENSORBOARD_LOG"
echo ""

# Activate venv and start TensorBoard
source venv/bin/activate
nohup tensorboard --logdir=logs/tensorboard --port=$TENSORBOARD_PORT --host=0.0.0.0 > "$TENSORBOARD_LOG" 2>&1 &
TENSORBOARD_PID=$!

echo "✅ TensorBoard started (PID: $TENSORBOARD_PID)"
echo ""
echo "📊 TensorBoard URL: http://localhost:$TENSORBOARD_PORT"
echo "📝 Log file: $TENSORBOARD_LOG"
echo ""
echo "💡 To stop TensorBoard:"
echo "   kill $TENSORBOARD_PID"
echo "   or: pkill -f 'tensorboard.*--port=$TENSORBOARD_PORT'"
echo ""

# Wait a bit and check if it's running
sleep 3
if ps -p $TENSORBOARD_PID > /dev/null 2>&1; then
    echo "✅ TensorBoard is running successfully!"
else
    echo "❌ TensorBoard failed to start. Check log: $TENSORBOARD_LOG"
    exit 1
fi


