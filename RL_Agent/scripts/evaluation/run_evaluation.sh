#!/bin/bash
# Convenience script to evaluate trained model
# Usage: ./scripts/run_evaluation.sh MODEL_PATH [--episodes N]

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

# Check arguments
if [ -z "$1" ]; then
    echo "Usage: $0 MODEL_PATH [--episodes N]"
    echo ""
    echo "Example:"
    echo "  $0 checkpoints/best_model.zip"
    echo "  $0 checkpoints/best_model.zip --episodes 20"
    exit 1
fi

MODEL_PATH="$1"
shift  # Remove first argument

# Activate venv
source venv/bin/activate

# Check if CARLA is running
if ! pgrep -f "CarlaUE4" > /dev/null; then
    echo "⚠️  CARLA simulator doesn't seem to be running"
    echo "   Please start CARLA in another terminal:"
    echo "   cd /home/a/Desktop/CARLA_0.9.16 && ./CarlaUE4.sh"
    exit 1
fi

# Run evaluation
echo "📊 Starting evaluation..."
echo ""

python training/evaluate.py --model "$MODEL_PATH" --config config/phase1_config.yaml "$@"

