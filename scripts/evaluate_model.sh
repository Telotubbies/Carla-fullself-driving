#!/bin/bash
# Evaluate trained SAC model script

set -e

echo "=========================================="
echo "Evaluating SAC Model on CARLA"
echo "=========================================="

# Activate virtual environment
if [ -d "venv" ]; then
    echo "Activating virtual environment..."
    source venv/bin/activate
else
    echo "Virtual environment not found. Please run setup first."
    exit 1
fi

# Check if CARLA server is running
echo "Checking CARLA server connection..."
python3 -c "
import carla
try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    client.get_server_version()
    print('CARLA server is running')
except:
    print('ERROR: CARLA server is not running!')
    print('Please start CARLA server first')
    exit(1)
"

if [ $? -ne 0 ]; then
    exit 1
fi

# Parse arguments
CHECKPOINT=${1:-"data/checkpoints/best_model"}
NUM_EPISODES=${2:-10}

echo "Checkpoint: $CHECKPOINT"
echo "Number of episodes: $NUM_EPISODES"
echo "=========================================="

# Run evaluation
python3 src/sac_trainer/evaluation.py "$CHECKPOINT" --episodes "$NUM_EPISODES"

echo "=========================================="
echo "Evaluation completed!"
echo "=========================================="
