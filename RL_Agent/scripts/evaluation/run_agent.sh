#!/bin/bash
# Script to run trained agent and watch it drive
# Usage: ./scripts/run_agent.sh [MODEL_PATH] [--episodes N]

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$( cd "$SCRIPT_DIR/.." && pwd )"

cd "$PROJECT_DIR"

# Check if venv exists
if [ ! -d "venv" ]; then
    echo "❌ Virtual environment not found!"
    echo "   Please run: ./setup_venv.sh"
    exit 1
fi

# Default model path (latest checkpoint)
if [ -z "$1" ] || [[ "$1" == --* ]]; then
    # Find latest checkpoint
    LATEST_MODEL=$(find checkpoints/checkpoint -name "rl_model_*_steps.zip" -type f | sort -V | tail -1)
    if [ -z "$LATEST_MODEL" ]; then
        echo "❌ No model found in checkpoints/checkpoint/"
        echo "   Please train a model first or specify model path"
        exit 1
    fi
    MODEL_PATH="$LATEST_MODEL"
    echo "📦 Using latest model: $MODEL_PATH"
else
    MODEL_PATH="$1"
    shift
fi

# Activate venv
source venv/bin/activate

# Check if CARLA is running
if ! pgrep -f "CarlaUE4" > /dev/null; then
    echo "⚠️  CARLA simulator doesn't seem to be running"
    echo "   Please start CARLA in another terminal:"
    echo "   cd /home/a/Desktop/CARLA_0.9.16 && ./CarlaUE4.sh"
    exit 1
fi

# Determine config file based on model path
if [[ "$MODEL_PATH" == *"faster"* ]] || [[ "$MODEL_PATH" == *"fast"* ]]; then
    CONFIG="config/phase1_faster_hyperparams.yaml"
elif [[ "$MODEL_PATH" == *"improved"* ]]; then
    CONFIG="config/phase1_improved.yaml"
else
    CONFIG="config/phase1_faster_hyperparams.yaml"
fi

# Run evaluation
echo "======================================================================"
echo "🚗 Running Trained Agent"
echo "======================================================================"
echo ""
echo "📦 Model: $MODEL_PATH"
echo "⚙️  Config: $CONFIG"
echo ""
echo "💡 รถจะวิ่งใน CARLA simulator"
echo "   - ดูใน CARLA window เพื่อดูรถวิ่ง"
echo "   - กด Ctrl+C เพื่อหยุด"
echo ""
echo "======================================================================"
echo ""

python training/evaluate.py --model "$MODEL_PATH" --config "$CONFIG" "$@"

