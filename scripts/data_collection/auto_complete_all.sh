#!/bin/bash
# Auto-complete pipeline: Training → Config Update → Inference

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

cd "$(dirname "$0")"

# Find latest data directory
LATEST_DATA=$(ls -td data/autopilot_* 2>/dev/null | head -1)

if [ -z "$LATEST_DATA" ]; then
    echo -e "${RED}❌ No data directory found!${NC}"
    exit 1
fi

echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}Auto-Complete Pipeline${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "Data directory: ${GREEN}$LATEST_DATA${NC}"
echo ""

# STEP 1: Train LSTM
echo -e "${BLUE}STEP 1: Training LSTM Model${NC}"
echo -e "${YELLOW}This may take a while...${NC}"
echo ""

python3 training/train_lstm.py "$LATEST_DATA" \
    --epochs 50 \
    --batch-size 32 \
    --lr 0.001 \
    --sequence-length 10 \
    --hidden-size 256 \
    --num-layers 2

if [ $? -ne 0 ]; then
    echo -e "${RED}❌ Training failed!${NC}"
    exit 1
fi

MODEL_PATH="$LATEST_DATA/lstm_model/best_model.pth"

if [ ! -f "$MODEL_PATH" ]; then
    echo -e "${RED}❌ Model not found: $MODEL_PATH${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Training complete!${NC}"
echo ""

# STEP 2: Update Config
echo -e "${BLUE}STEP 2: Updating config.yaml${NC}"

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

# STEP 3: Run Inference
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}STEP 3: Running Inference${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${YELLOW}Starting inference (press Ctrl+C to stop)...${NC}"
echo ""

python3 main.py --mode inference

echo ""
echo -e "${GREEN}════════════════════════════════════════${NC}"
echo -e "${GREEN}✅ Complete Pipeline Finished!${NC}"
echo -e "${GREEN}════════════════════════════════════════${NC}"

