#!/bin/bash
# Complete Auto Pipeline: ตั้งแต่ต้นจนจบ - Auto Complete
# 1. Collect Data (ถ้ายังไม่มี)
# 2. Extract Features (ถ้ายังไม่มี)
# 3. Train LSTM (ถ้ายังไม่มี)
# 4. Update Config
# 5. Run Inference with GUI

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

cd "$(dirname "$0")"

# Set ROCm environment
export HSA_OVERRIDE_GFX_VERSION=11.0.0
export AMD_SERIALIZE_KERNEL=3
export HIP_FORCE_DEV_KERNELS=1

# CARLA environment
export CARLA_DIR=/home/a/Desktop/CARLA_0.9.16
CARLA_EGG=$(find "$CARLA_DIR/PythonAPI/carla/dist" -name "carla-*.egg" 2>/dev/null | head -1)
export PYTHONPATH="$CARLA_EGG:$PYTHONPATH"

echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}🚀 Complete Auto Pipeline${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo ""

# Find latest data directory or create new one
LATEST_DATA=$(ls -td data/autopilot_* 2>/dev/null | head -1)

# STEP 1: Check if we need to collect data
if [ -z "$LATEST_DATA" ] || [ ! -f "$LATEST_DATA/data.csv" ] || [ $(wc -l < "$LATEST_DATA/data.csv" 2>/dev/null || echo 0) -lt 1000 ]; then
    echo -e "${YELLOW}STEP 1: Collecting Data${NC}"
    echo -e "${YELLOW}This may take a while (20,000 frames)...${NC}"
    echo ""
    
    python3 training/collect_autopilot_data.py --frames 20000
    
    if [ $? -ne 0 ]; then
        echo -e "${RED}❌ Data collection failed!${NC}"
        exit 1
    fi
    
    LATEST_DATA=$(ls -td data/autopilot_* 2>/dev/null | head -1)
    echo -e "${GREEN}✅ Data collection complete!${NC}"
    echo ""
else
    echo -e "${GREEN}✅ Data already exists: $LATEST_DATA${NC}"
    echo ""
fi

# STEP 2: Check if we need to extract features
if [ ! -f "$LATEST_DATA/features.npy" ]; then
    echo -e "${YELLOW}STEP 2: Extracting Features${NC}"
    echo -e "${YELLOW}This may take a while...${NC}"
    echo ""
    
    python3 training/extract_features.py --data_dir "$LATEST_DATA" --preprocess
    
    if [ $? -ne 0 ]; then
        echo -e "${RED}❌ Feature extraction failed!${NC}"
        exit 1
    fi
    
    echo -e "${GREEN}✅ Feature extraction complete!${NC}"
    echo ""
else
    echo -e "${GREEN}✅ Features already exist${NC}"
    echo ""
fi

# STEP 3: Check if we need to preprocess data
if [ ! -f "$LATEST_DATA/processed/data_processed.csv" ]; then
    echo -e "${YELLOW}STEP 2.5: Preprocessing Data${NC}"
    python3 << EOF
from training.data_preprocessing import preprocess_dataset
from pathlib import Path
import logging
logging.basicConfig(level=logging.INFO)

preprocess_dataset('$LATEST_DATA')
print("✅ Preprocessing complete")
EOF
    echo ""
fi

# STEP 4: Check if we need to train LSTM
MODEL_PATH="$LATEST_DATA/lstm_model/best_model.pth"
if [ ! -f "$MODEL_PATH" ]; then
    echo -e "${YELLOW}STEP 3: Training LSTM Model${NC}"
    echo -e "${YELLOW}This will take a while (50 epochs)...${NC}"
    echo ""
    
    # Check if training is already running
    if pgrep -f "train_lstm.py" > /dev/null; then
        echo -e "${YELLOW}Training already running, waiting for completion...${NC}"
        while pgrep -f "train_lstm.py" > /dev/null; do
            echo -n "."
            sleep 10
        done
        echo ""
    else
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
    fi
    
    # Wait for model file
    echo -e "${YELLOW}Waiting for model file...${NC}"
    for i in {1..60}; do
        if [ -f "$MODEL_PATH" ]; then
            break
        fi
        echo -n "."
        sleep 5
    done
    echo ""
    
    if [ ! -f "$MODEL_PATH" ]; then
        echo -e "${RED}❌ Model not found after training!${NC}"
        exit 1
    fi
    
    echo -e "${GREEN}✅ Training complete!${NC}"
    echo ""
else
    echo -e "${GREEN}✅ Trained model already exists${NC}"
    echo ""
fi

# STEP 5: Update config.yaml
echo -e "${BLUE}STEP 4: Updating config.yaml${NC}"
python3 << EOF
import yaml

with open('config.yaml', 'r') as f:
    config = yaml.safe_load(f)

config['temporal']['trained_model_path'] = '$MODEL_PATH'

with open('config.yaml', 'w') as f:
    yaml.dump(config, f, default_flow_style=False, sort_keys=False)

print("✅ Updated config.yaml with trained model path: $MODEL_PATH")
EOF

echo -e "${GREEN}✅ Configuration updated${NC}"
echo ""

# STEP 6: Run Inference with GUI
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}STEP 5: Running Inference with GUI${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${GREEN}Starting autonomous driving system...${NC}"
echo -e "${YELLOW}GUI will open shortly. Press Ctrl+C to stop.${NC}"
echo ""

# Ensure CARLA is running
if ! pgrep -f "CarlaUE4" > /dev/null; then
    echo -e "${YELLOW}⚠️  CARLA not running. Please start CARLA first!${NC}"
    echo -e "${YELLOW}   Run: cd $CARLA_DIR && ./CarlaUE4.sh${NC}"
    exit 1
fi

# Run inference
python3 main.py --mode inference

echo ""
echo -e "${GREEN}════════════════════════════════════════${NC}"
echo -e "${GREEN}✅ Complete Pipeline Finished!${NC}"
echo -e "${GREEN}════════════════════════════════════════${NC}"
echo ""
echo "Summary:"
echo "  Data directory: $LATEST_DATA"
echo "  Trained model: $MODEL_PATH"
echo "  Config updated: config.yaml"
echo ""

