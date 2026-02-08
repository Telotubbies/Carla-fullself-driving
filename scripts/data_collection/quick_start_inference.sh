#!/bin/bash
# Quick start inference (assumes training is done)

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

cd "$(dirname "$0")"

# Set environment
export HSA_OVERRIDE_GFX_VERSION=11.0.0
export CARLA_DIR=/home/a/Desktop/CARLA_0.9.16
CARLA_EGG=$(find "$CARLA_DIR/PythonAPI/carla/dist" -name "carla-*.egg" 2>/dev/null | head -1)
export PYTHONPATH="$CARLA_EGG:$PYTHONPATH"

echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}🚀 Quick Start Inference${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo ""

# Find latest model
LATEST_DATA=$(ls -td data/autopilot_* 2>/dev/null | head -1)
MODEL_PATH="$LATEST_DATA/lstm_model/best_model.pth"

if [ ! -f "$MODEL_PATH" ]; then
    echo -e "${RED}❌ Model not found: $MODEL_PATH${NC}"
    echo -e "${YELLOW}   Please run training first!${NC}"
    exit 1
fi

# Update config
echo -e "${YELLOW}Updating config.yaml...${NC}"
python3 << EOF
import yaml

with open('config.yaml', 'r') as f:
    config = yaml.safe_load(f)

config['temporal']['trained_model_path'] = '$MODEL_PATH'

with open('config.yaml', 'w') as f:
    yaml.dump(config, f, default_flow_style=False, sort_keys=False)

print("✅ Updated config.yaml")
EOF

# Check CARLA
if ! pgrep -f "CarlaUE4" > /dev/null; then
    echo -e "${RED}❌ CARLA is not running!${NC}"
    echo -e "${YELLOW}   Please start CARLA first:${NC}"
    echo -e "${YELLOW}   cd $CARLA_DIR && ./CarlaUE4.sh${NC}"
    exit 1
fi

echo -e "${GREEN}✅ CARLA is running${NC}"
echo ""

# Start inference
echo -e "${BLUE}Starting inference with GUI...${NC}"
echo -e "${YELLOW}GUI will open shortly. Press Ctrl+C to stop.${NC}"
echo ""

python3 main.py --mode inference

