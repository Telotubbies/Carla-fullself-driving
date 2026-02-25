#!/bin/bash
# Quick script to run MPC inference

cd "$(dirname "$0")"

# Set ROCm environment
export HSA_OVERRIDE_GFX_VERSION=11.0.0
export AMD_SERIALIZE_KERNEL=3
export HIP_FORCE_DEV_KERNELS=1

# CARLA environment
export CARLA_DIR=/home/supawich/Desktop/CARLA_0.9.16
CARLA_EGG=$(find "$CARLA_DIR/PythonAPI/carla/dist" -name "carla-*.egg" 2>/dev/null | head -1)
export PYTHONPATH="$CARLA_EGG:$PYTHONPATH"

echo "🚗 Starting MPC Inference..."
echo ""

# Check if CARLA is running
if ! pgrep -f "CarlaUE4" > /dev/null; then
    echo "⚠️  CARLA is not running!"
    echo "   Starting CARLA..."
    cd "$CARLA_DIR"
    ./CarlaUE4.sh -quality-level=Low -prefernoloadscreen > /tmp/carla.log 2>&1 &
    echo "⏳ Waiting 40 seconds for CARLA to load..."
    sleep 40
    cd "$(dirname "$0")"
fi

# Check if model exists
MODEL_PATH="data/autopilot_20260208_150902/lstm_model/best_model.pth"
if [ ! -f "$MODEL_PATH" ]; then
    echo "❌ Model not found: $MODEL_PATH"
    echo "   Please train the model first or update config.yaml"
    exit 1
fi

echo "✅ CARLA is running"
echo "✅ Model found: $MODEL_PATH"
echo ""
echo "Starting inference... (Press Ctrl+C to stop)"
echo ""

# Run inference
python3 main.py --mode inference
