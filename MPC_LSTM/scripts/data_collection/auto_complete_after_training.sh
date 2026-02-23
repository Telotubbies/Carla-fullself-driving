#!/bin/bash
# Auto-complete after training: Update Config → Run Inference

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

cd "$(dirname "$0")"

# Find latest data directory
LATEST_DATA=$(ls -td data/autopilot_* 2>/dev/null | head -1)

if [ -z "$LATEST_DATA" ]; then
    echo -e "${RED}❌ No data directory found!${NC}"
    exit 1
fi

MODEL_PATH="$LATEST_DATA/lstm_model/best_model.pth"

# Wait for training to complete
echo -e "${YELLOW}Waiting for training to complete...${NC}"
while pgrep -f "train_lstm.py" > /dev/null; do
    echo -n "."
    sleep 10
done
echo ""
echo -e "${GREEN}✅ Training completed!${NC}"

# Check if model exists
if [ ! -f "$MODEL_PATH" ]; then
    echo -e "${RED}❌ Model not found: $MODEL_PATH${NC}"
    exit 1
fi

echo -e "${BLUE}STEP 1: Updating config.yaml${NC}"
python3 << EOF
import yaml

with open('config.yaml', 'r') as f:
    config = yaml.safe_load(f)

config['temporal']['trained_model_path'] = '$MODEL_PATH'

with open('config.yaml', 'w') as f:
    yaml.dump(config, f, default_flow_style=False, sort_keys=False)

print("✅ Updated config.yaml with trained model path")
EOF

echo -e "${GREEN}✅ Configuration updated${NC}"
echo ""

# STEP 2: Run Inference
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}STEP 2: Running Inference${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${YELLOW}Starting inference (press Ctrl+C to stop)...${NC}"
echo ""

export HSA_OVERRIDE_GFX_VERSION=11.0.0
export CARLA_DIR=/home/a/Desktop/CARLA_0.9.16
CARLA_EGG=$(find "$CARLA_DIR/PythonAPI/carla/dist" -name "carla-*.egg" 2>/dev/null | head -1)
export PYTHONPATH="$CARLA_EGG:$PYTHONPATH"

python3 main.py --mode inference

echo ""
echo -e "${GREEN}════════════════════════════════════════${NC}"
echo -e "${GREEN}✅ Complete Pipeline Finished!${NC}"
echo -e "${GREEN}════════════════════════════════════════${NC}"

