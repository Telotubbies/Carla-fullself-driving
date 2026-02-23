#!/bin/bash
# Fix issues and continue pipeline from current state

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CARLA_DIR="${CARLA_DIR:-/home/a/Desktop/CARLA_0.9.16}"

echo -e "${BLUE}🔧 Fix and Continue Pipeline${NC}"
echo "=============================="
echo ""

# Setup environment
export HSA_OVERRIDE_GFX_VERSION=11.0.0
CARLA_EGG=$(find "$CARLA_DIR/PythonAPI/carla/dist" -name "carla-*.egg" 2>/dev/null | head -1)
if [ -n "$CARLA_EGG" ]; then
    export PYTHONPATH="$CARLA_EGG:$PYTHONPATH"
fi

cd "$PROJECT_DIR"

# Find latest data directory
DATA_DIR=$(ls -td data/autopilot_* 2>/dev/null | head -1)

if [ -z "$DATA_DIR" ]; then
    echo -e "${RED}❌ No data directory found${NC}"
    exit 1
fi

echo -e "${BLUE}Found data directory: $DATA_DIR${NC}"

# Check current data
CURRENT_FRAMES=$(wc -l < "$DATA_DIR/data.csv")
echo -e "${YELLOW}Current frames: $((CURRENT_FRAMES-1))${NC}"

# Check if we need to continue collection
TARGET_FRAMES=20000
if [ $((CURRENT_FRAMES-1)) -lt $TARGET_FRAMES ]; then
    REMAINING=$((TARGET_FRAMES - CURRENT_FRAMES + 1))
    echo -e "${YELLOW}Need to collect $REMAINING more frames${NC}"
    echo -e "${BLUE}Continuing data collection...${NC}"
    python3 training/collect_autopilot_data.py --frames "$REMAINING" --output "$DATA_DIR"
else
    echo -e "${GREEN}✅ Sufficient data collected${NC}"
fi

# Check CARLA
echo -e "${YELLOW}Checking CARLA...${NC}"
if ! python3 -c "import carla; client = carla.Client('localhost', 2000); client.set_timeout(5.0); client.get_world()" 2>/dev/null; then
    echo -e "${RED}❌ CARLA not accessible${NC}"
    exit 1
fi

# Continue from preprocessing
echo ""
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}STEP 2: Preprocessing Data${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
if [ ! -d "$DATA_DIR/processed" ]; then
    python3 -c "from training.data_preprocessing import preprocess_dataset; preprocess_dataset('$DATA_DIR')"
    echo -e "${GREEN}✅ Preprocessing complete${NC}"
else
    echo -e "${GREEN}✅ Already preprocessed${NC}"
fi

echo ""
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}STEP 3: Extracting Features${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
if [ ! -f "$DATA_DIR/features.npy" ]; then
    python3 training/extract_features.py "$DATA_DIR"
    echo -e "${GREEN}✅ Feature extraction complete${NC}"
else
    echo -e "${GREEN}✅ Features already extracted${NC}"
fi

echo ""
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}STEP 4: Training LSTM${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
if [ ! -f "$DATA_DIR/lstm_model/best_model.pth" ]; then
    EPOCHS=${1:-30}
    python3 training/train_lstm.py "$DATA_DIR" \
        --output "$DATA_DIR/lstm_model" \
        --epochs "$EPOCHS" \
        --batch-size 64
    echo -e "${GREEN}✅ Training complete${NC}"
else
    echo -e "${GREEN}✅ Model already trained${NC}"
fi

echo ""
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}STEP 5: Updating Config${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
MODEL_PATH="$DATA_DIR/lstm_model/best_model.pth"
if [ -f "$MODEL_PATH" ]; then
    python3 << EOF
import yaml

with open('config.yaml', 'r') as f:
    config = yaml.safe_load(f)

config['temporal']['trained_model_path'] = '$MODEL_PATH'

with open('config.yaml', 'w') as f:
    yaml.dump(config, f, default_flow_style=False, sort_keys=False)

print("✅ Config updated")
EOF
else
    echo -e "${RED}❌ Model not found${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}════════════════════════════════════════${NC}"
echo -e "${GREEN}✅ Pipeline Complete!${NC}"
echo -e "${GREEN}════════════════════════════════════════${NC}"
echo ""
echo "Ready to run inference:"
echo "  python3 main.py --mode inference"

