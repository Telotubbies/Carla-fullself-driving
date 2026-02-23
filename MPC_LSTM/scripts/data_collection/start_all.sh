#!/bin/bash
# Start all: CARLA + Data Collection

set -e

export HSA_OVERRIDE_GFX_VERSION=11.0.0
export CARLA_DIR=/home/a/Desktop/CARLA_0.9.16
CARLA_EGG=$(find "$CARLA_DIR/PythonAPI/carla/dist" -name "carla-*.egg" 2>/dev/null | head -1)
export PYTHONPATH="$CARLA_EGG:$PYTHONPATH"

cd "$(dirname "$0")"

echo "🚀 Starting All: CARLA + Data Collection"
echo "========================================"
echo ""

# Step 1: Start CARLA if not running
if ! pgrep -f "CarlaUE4" > /dev/null; then
    echo "Step 1: Starting CARLA..."
    cd "$CARLA_DIR"
    ./CarlaUE4.sh -quality-level=Low -prefernoloadscreen > /tmp/carla.log 2>&1 &
    CARLA_PID=$!
    echo "   CARLA PID: $CARLA_PID"
    echo "   ⏳ Waiting for CARLA to start (30 seconds)..."
    sleep 30
else
    echo "✅ CARLA is already running"
fi

# Step 2: Wait for CARLA to be ready
echo ""
echo "Step 2: Checking CARLA connection..."
for i in {1..10}; do
    if timeout 5 python3 -c "import carla; c=carla.Client('localhost',2000); c.set_timeout(3); c.get_world()" 2>/dev/null; then
        echo "   ✅ CARLA is ready!"
        break
    else
        echo "   ⏳ Waiting... ($i/10)"
        sleep 3
    fi
done

# Step 3: Start data collection
echo ""
echo "Step 3: Starting data collection..."
cd "$(dirname "$0")"
python3 training/collect_autopilot_data.py --frames 20000 > data_collection.log 2>&1 &
COLLECT_PID=$!
echo "   Collection PID: $COLLECT_PID"
echo "   Log: tail -f data_collection.log"
echo ""
echo "✅ All started!"
echo ""
echo "Monitor:"
echo "   tail -f data_collection.log"
echo "   ./check_pipeline.sh"
