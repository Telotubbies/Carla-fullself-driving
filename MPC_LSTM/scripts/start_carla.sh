#!/bin/bash
# Start CARLA Simulator

CARLA_DIR="/home/a/Desktop/CARLA_0.9.16"
CARLA_EXEC="$CARLA_DIR/CarlaUE4.sh"

echo "🚀 Starting CARLA Simulator..."
echo "========================================"

# Check if CARLA is already running
if netstat -tuln 2>/dev/null | grep -q ":2000 " || ss -tuln 2>/dev/null | grep -q ":2000 "; then
    echo "⚠️  CARLA is already running on port 2000"
    exit 1
fi

# Check if executable exists
if [ ! -f "$CARLA_EXEC" ]; then
    echo "❌ CARLA executable not found: $CARLA_EXEC"
    exit 1
fi

# Start CARLA
cd "$CARLA_DIR"
echo "📂 Working directory: $CARLA_DIR"
echo "🎮 Starting CARLA..."
echo ""
echo "💡 Tips:"
echo "   - CARLA will take 30-60 seconds to load"
echo "   - Check logs for any errors"
echo "   - Press Ctrl+C to stop"
echo ""

# Start in background or foreground based on argument
if [ "$1" == "--background" ] || [ "$1" == "-b" ]; then
    echo "🔄 Starting in background..."
    nohup "$CARLA_EXEC" > /tmp/carla.log 2>&1 &
    CARLA_PID=$!
    echo "✅ CARLA started (PID: $CARLA_PID)"
    echo "📝 Logs: /tmp/carla.log"
    echo ""
    echo "⏳ Waiting for CARLA to be ready..."
    sleep 5
    
    # Wait for port to be available
    for i in {1..30}; do
        if netstat -tuln 2>/dev/null | grep -q ":2000 " || ss -tuln 2>/dev/null | grep -q ":2000 "; then
            echo "✅ CARLA is ready on port 2000"
            exit 0
        fi
        echo "   Waiting... ($i/30)"
        sleep 2
    done
    
    echo "⚠️  CARLA may still be loading. Check /tmp/carla.log"
else
    # Start in foreground
    "$CARLA_EXEC"
fi

