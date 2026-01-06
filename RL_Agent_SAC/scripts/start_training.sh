#!/bin/bash
# Start SAC Training Script

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_DIR"

echo "=========================================="
echo "🚀 Starting SAC Training"
echo "=========================================="
echo ""

# Default values
CONFIG="${1:-config/sac_config.yaml}"
RESUME="${2:-}"
NUM_ENVS="${3:-1}"

# Build command
CMD="python training/train_sac.py --config $CONFIG --num-envs $NUM_ENVS"

if [ -n "$RESUME" ]; then
    CMD="$CMD --resume $RESUME"
fi

echo "Configuration: $CONFIG"
echo "Resume from: ${RESUME:-None}"
echo "Number of environments: $NUM_ENVS"
echo ""
echo "Command: $CMD"
echo ""
echo "=========================================="
echo ""

# Run training
$CMD

