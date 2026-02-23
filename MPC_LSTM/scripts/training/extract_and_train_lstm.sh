#!/bin/bash
# Extract Features and Train LSTM Sequence Script

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

cd "$(dirname "$0")/../.."

# Set ROCm environment
export HSA_OVERRIDE_GFX_VERSION=11.0.0
export AMD_SERIALIZE_KERNEL=3
export HIP_FORCE_DEV_KERNELS=1

# CARLA environment
export CARLA_DIR=/home/a/Desktop/CARLA_0.9.16
CARLA_EGG=$(find "$CARLA_DIR/PythonAPI/carla/dist" -name "carla-*.egg" 2>/dev/null | head -1)
export PYTHONPATH="$CARLA_EGG:$PYTHONPATH"

echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}🔄 Extract Features + Train LSTM Sequence${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo ""

# Default values
DATA_DIR="${1:-data/autopilot_20260208_150902}"
LANE_MODEL="${2:-data/autopilot_20260208_150902/lane_unet_model/lane_unet_final.pth}"
BATCH_SIZE="${3:-32}"
EPOCHS="${4:-100}"

# Check if data directory exists
if [ ! -d "$DATA_DIR" ]; then
    echo -e "${RED}❌ Data directory not found: $DATA_DIR${NC}"
    exit 1
fi

echo -e "${GREEN}📋 Configuration:${NC}"
echo "   Data directory: $DATA_DIR"
echo "   Lane model: $LANE_MODEL"
echo "   Batch size: $BATCH_SIZE"
echo "   Training epochs: $EPOCHS"
echo ""

# Step 1: Extract Features
echo -e "${YELLOW}════════════════════════════════════════${NC}"
echo -e "${YELLOW}Step 1: Extracting Features (ResNet + Lane)${NC}"
echo -e "${YELLOW}════════════════════════════════════════${NC}"
echo ""

python3 training/extract_features.py \
    --data-dir "$DATA_DIR" \
    --use-lane-features \
    --lane-model "$LANE_MODEL" \
    --batch-size "$BATCH_SIZE"

if [ $? -ne 0 ]; then
    echo -e "${RED}❌ Feature extraction failed!${NC}"
    exit 1
fi

# Verify features
echo ""
echo -e "${GREEN}✅ Verifying features...${NC}"
FEATURES_SHAPE=$(python3 -c "import numpy as np; f = np.load('$DATA_DIR/features.npy'); print(f.shape)" 2>/dev/null)
echo "   Features shape: $FEATURES_SHAPE"

if [[ ! "$FEATURES_SHAPE" == *"640"* ]]; then
    echo -e "${RED}❌ Expected 640-dim features, got: $FEATURES_SHAPE${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Feature extraction complete!${NC}"
echo ""

# Step 2: Train LSTM
echo -e "${YELLOW}════════════════════════════════════════${NC}"
echo -e "${YELLOW}Step 2: Training LSTM with 640-dim features${NC}"
echo -e "${YELLOW}════════════════════════════════════════${NC}"
echo ""

python3 training/train_lstm.py \
    "$DATA_DIR" \
    --output "$DATA_DIR/lstm_model" \
    --use-attention \
    --use-advanced-loss \
    --epochs "$EPOCHS" \
    --batch-size 64 \
    --gradient-clip 1.0 \
    --early-stopping 50 \
    --lr 0.001

if [ $? -ne 0 ]; then
    echo -e "${RED}❌ LSTM training failed!${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}✅ LSTM training complete!${NC}"
echo ""

# Step 3: Update config.yaml
echo -e "${YELLOW}════════════════════════════════════════${NC}"
echo -e "${YELLOW}Step 3: Updating config.yaml${NC}"
echo -e "${YELLOW}════════════════════════════════════════${NC}"
echo ""

CONFIG_FILE="config.yaml"
if [ -f "$CONFIG_FILE" ]; then
    # Backup config
    cp "$CONFIG_FILE" "${CONFIG_FILE}.bak"
    
    # Update temporal.input_size to 640
    python3 << EOF
import yaml
import sys

with open('$CONFIG_FILE', 'r') as f:
    config = yaml.safe_load(f)

config['temporal']['input_size'] = 640
config['temporal']['trained_model_path'] = '$DATA_DIR/lstm_model/best_model.pth'

with open('$CONFIG_FILE', 'w') as f:
    yaml.dump(config, f, default_flow_style=False, sort_keys=False)

print("✅ Updated config.yaml:")
print(f"   temporal.input_size: 640")
print(f"   temporal.trained_model_path: $DATA_DIR/lstm_model/best_model.pth")
EOF
    
    echo -e "${GREEN}✅ Config updated!${NC}"
else
    echo -e "${YELLOW}⚠️  config.yaml not found, skipping update${NC}"
fi

echo ""
echo -e "${GREEN}════════════════════════════════════════${NC}"
echo -e "${GREEN}✅ All steps complete!${NC}"
echo -e "${GREEN}════════════════════════════════════════${NC}"
echo ""
echo "📁 Output files:"
echo "   - Features: $DATA_DIR/features.npy (640 dim)"
echo "   - LSTM model: $DATA_DIR/lstm_model/best_model.pth"
echo "   - Config backup: ${CONFIG_FILE}.bak"
echo ""
echo "💡 Ready for inference:"
echo "   python3 main.py --mode inference"
echo ""

