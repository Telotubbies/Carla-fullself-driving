#!/bin/bash
# Quick run script with ROCm fix for AMD 7800XT

# Set ROCm environment for gfx1101 compatibility
export HSA_OVERRIDE_GFX_VERSION=11.0.0

# Get CARLA Python API
CARLA_DIR="${CARLA_DIR:-/home/a/Desktop/CARLA_0.9.16}"
CARLA_EGG=$(find "$CARLA_DIR/PythonAPI/carla/dist" -name "carla-*.egg" 2>/dev/null | head -1)

if [ -n "$CARLA_EGG" ]; then
    export PYTHONPATH="$CARLA_EGG:$PYTHONPATH"
fi

# Run
cd "$(dirname "$0")"
python3 main.py "$@"

