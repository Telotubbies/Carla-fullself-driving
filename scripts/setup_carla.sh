#!/bin/bash
# Setup script for CARLA connection

set -e

echo "=========================================="
echo "CARLA Setup Script"
echo "=========================================="

# Check if CARLA is installed
CARLA_PATH=${CARLA_PATH:-"/opt/carla-simulator"}

if [ ! -d "$CARLA_PATH" ]; then
    echo "CARLA not found at $CARLA_PATH"
    echo "Please set CARLA_PATH environment variable or install CARLA"
    exit 1
fi

echo "CARLA found at: $CARLA_PATH"

# Check CARLA version
if [ -f "$CARLA_PATH/VERSION" ]; then
    CARLA_VERSION=$(cat "$CARLA_PATH/VERSION")
    echo "CARLA Version: $CARLA_VERSION"
fi

# Setup Python API path
export PYTHONPATH=$PYTHONPATH:$CARLA_PATH/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg

echo "PYTHONPATH updated"

# Test CARLA Python API
echo "Testing CARLA Python API..."
python3 -c "import carla; print('CARLA Python API imported successfully')" || {
    echo "Failed to import CARLA Python API"
    echo "Please check your CARLA installation"
    exit 1
}

echo "=========================================="
echo "CARLA setup completed successfully!"
echo "=========================================="
