#!/bin/bash
# Complete Training Pipeline Script

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CARLA_DIR="${CARLA_DIR:-/home/a/Desktop/CARLA_0.9.16}"

echo -e "${BLUE}🎓 Complete Training Pipeline${NC}"
echo "================================"
echo ""

# Setup environment
export HSA_OVERRIDE_GFX_VERSION=11.0.0
CARLA_EGG=$(find "$CARLA_DIR/PythonAPI/carla/dist" -name "carla-*.egg" 2>/dev/null | head -1)
if [ -n "$CARLA_EGG" ]; then
    export PYTHONPATH="$CARLA_EGG:$PYTHONPATH"
fi

cd "$PROJECT_DIR"

# STEP 1: Collect Data
echo -e "${BLUE}STEP 1: Collecting Data with Autopilot...${NC}"
echo -e "${YELLOW}Make sure CARLA is running!${NC}"
read -p "Press Enter when CARLA is ready..."

FRAMES=${1:-20000}
python3 training/collect_autopilot_data.py --frames "$FRAMES"
DATA_DIR=$(ls -td data/autopilot_* | head -1)

echo ""
echo -e "${GREEN}✅ Data collection complete: $DATA_DIR${NC}"
echo ""

# STEP 2: Preprocess Data (Clean, Normalize, etc.)
echo -e "${BLUE}STEP 2: Preprocessing Data (Clean, Normalize, Outlier Detection)...${NC}"
python3 -c "from training.data_preprocessing import preprocess_dataset; preprocess_dataset('$DATA_DIR')"
echo -e "${GREEN}✅ Data preprocessing complete${NC}"
echo ""

# STEP 3: Extract Features
echo -e "${BLUE}STEP 3: Extracting Features...${NC}"
python3 training/extract_features.py "$DATA_DIR"
echo -e "${GREEN}✅ Feature extraction complete${NC}"
echo ""

# STEP 4: Train LSTM
echo -e "${BLUE}STEP 4: Training LSTM...${NC}"

python3 training/train_lstm.py "$DATA_DIR" \
    --output "$DATA_DIR/lstm_model" \
    --epochs 50 \
    --batch-size 64

echo ""
echo -e "${GREEN}✅ Training complete!${NC}"
echo ""
echo -e "${BLUE}Model saved to: $DATA_DIR/lstm_model/best_model.pth${NC}"
echo ""
echo "To use the trained model, update config.yaml:"
echo "  temporal:"
echo "    trained_model_path: \"$DATA_DIR/lstm_model/best_model.pth\""

