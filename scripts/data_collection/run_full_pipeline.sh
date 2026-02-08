#!/bin/bash
# Complete Pipeline: Data Collection → Preprocess → Extract → Train → Inference

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CARLA_DIR="${CARLA_DIR:-/home/a/Desktop/CARLA_0.9.16}"

echo -e "${BLUE}🚗 Complete Training & Inference Pipeline${NC}"
echo "=============================================="
echo ""

# Setup environment
export HSA_OVERRIDE_GFX_VERSION=11.0.0
CARLA_EGG=$(find "$CARLA_DIR/PythonAPI/carla/dist" -name "carla-*.egg" 2>/dev/null | head -1)
if [ -n "$CARLA_EGG" ]; then
    export PYTHONPATH="$CARLA_EGG:$PYTHONPATH"
fi

cd "$PROJECT_DIR"

# Parameters
FRAMES=${1:-20000}
EPOCHS=${2:-30}

echo -e "${BLUE}Parameters:${NC}"
echo "  Frames to collect: $FRAMES"
echo "  Training epochs: $EPOCHS"
echo ""

# Check CARLA
echo -e "${YELLOW}Checking CARLA connection...${NC}"
if ! python3 -c "import carla; client = carla.Client('localhost', 2000); client.set_timeout(5.0); client.get_world()" 2>/dev/null; then
    echo -e "${RED}❌ CARLA not running or not accessible${NC}"
    echo "Please start CARLA first:"
    echo "  cd $CARLA_DIR && ./CarlaUE4.sh"
    exit 1
fi
echo -e "${GREEN}✅ CARLA is ready${NC}"
echo ""

# STEP 1: Collect Data
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}STEP 1: Collecting Data with Autopilot${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
python3 training/collect_autopilot_data.py --frames "$FRAMES"
DATA_DIR=$(ls -td data/autopilot_* | head -1)
echo -e "${GREEN}✅ Data collection complete: $DATA_DIR${NC}"
echo ""

# STEP 2: Preprocess Data
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}STEP 2: Preprocessing Data${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
python3 -c "from training.data_preprocessing import preprocess_dataset; preprocess_dataset('$DATA_DIR')"
echo -e "${GREEN}✅ Data preprocessing complete${NC}"
echo ""

# STEP 3: Extract Features
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}STEP 3: Extracting Features${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
python3 training/extract_features.py "$DATA_DIR"
echo -e "${GREEN}✅ Feature extraction complete${NC}"
echo ""

# STEP 4: Train LSTM
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}STEP 4: Training LSTM${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
python3 training/train_lstm.py "$DATA_DIR" \
    --output "$DATA_DIR/lstm_model" \
    --epochs "$EPOCHS" \
    --batch-size 64
echo -e "${GREEN}✅ LSTM training complete${NC}"
echo ""

# STEP 5: Update Config
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}STEP 5: Updating Configuration${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
MODEL_PATH="$DATA_DIR/lstm_model/best_model.pth"
if [ -f "$MODEL_PATH" ]; then
    # Backup original config
    cp config.yaml config.yaml.backup
    
    # Update config with trained model path
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
else
    echo -e "${RED}❌ Model not found: $MODEL_PATH${NC}"
    exit 1
fi
echo ""

# STEP 6: Test Inference
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}STEP 6: Testing Inference with Trained Model${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${YELLOW}Starting inference (press Ctrl+C to stop)...${NC}"
echo ""

python3 main.py --mode inference

echo ""
echo -e "${GREEN}════════════════════════════════════════${NC}"
echo -e "${GREEN}✅ Complete Pipeline Finished!${NC}"
echo -e "${GREEN}════════════════════════════════════════${NC}"
echo ""
echo "Summary:"
echo "  Data directory: $DATA_DIR"
echo "  Trained model: $MODEL_PATH"
echo "  Config updated: config.yaml"
echo ""

