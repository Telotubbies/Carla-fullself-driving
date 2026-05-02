#!/bin/bash
# Start SAC training script

set -e

echo "=========================================="
echo "Starting SAC Training on CARLA"
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
CONFIG_FILE=${1:-"config/sac_config.yaml"}

echo "Using configuration: $CONFIG_FILE"
echo "=========================================="

# Start training
python3 src/sac_trainer/train.py --config "$CONFIG_FILE"

echo "=========================================="
echo "Training completed!"
echo "=========================================="
