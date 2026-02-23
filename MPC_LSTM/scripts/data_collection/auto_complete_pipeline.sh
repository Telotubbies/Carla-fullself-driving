#!/bin/bash
# Auto-complete pipeline: Monitor and continue automatically

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CARLA_DIR="${CARLA_DIR:-/home/a/Desktop/CARLA_0.9.16}"

export HSA_OVERRIDE_GFX_VERSION=11.0.0
CARLA_EGG=$(find "$CARLA_DIR/PythonAPI/carla/dist" -name "carla-*.egg" 2>/dev/null | head -1)
if [ -n "$CARLA_EGG" ]; then
    export PYTHONPATH="$CARLA_EGG:$PYTHONPATH"
fi

cd "$PROJECT_DIR"

echo -e "${BLUE}🤖 Auto-Complete Pipeline${NC}"
echo "=========================="
echo ""

# Wait for data collection to complete
echo -e "${YELLOW}Waiting for data collection to complete...${NC}"
while pgrep -f "collect_autopilot_data" > /dev/null; do
    sleep 10
    # Check progress
    DATA_DIR=$(ls -td data/autopilot_* 2>/dev/null | head -1)
    if [ -n "$DATA_DIR" ] && [ -f "$DATA_DIR/data.csv" ]; then
        ROWS=$(wc -l < "$DATA_DIR/data.csv")
        echo -e "  Progress: $((ROWS-1)) frames collected..."
    fi
done

echo -e "${GREEN}✅ Data collection complete${NC}"

# Get latest data directory
DATA_DIR=$(ls -td data/autopilot_* 2>/dev/null | head -1)
if [ -z "$DATA_DIR" ]; then
    echo -e "${RED}❌ No data directory found${NC}"
    exit 1
fi

echo -e "${BLUE}Using data: $DATA_DIR${NC}"
echo ""

# Check data quality
echo -e "${BLUE}Checking data quality...${NC}"
python3 << EOF
import pandas as pd
import sys

df = pd.read_csv('$DATA_DIR/data.csv')
distance = ((df['x'].diff()**2 + df['y'].diff()**2)**0.5).sum()
moving = (df['velocity'] > 1.0).sum()

print(f"Total frames: {len(df)}")
print(f"Distance: {distance:.2f}m")
print(f"Moving frames: {moving}")

if distance < 100:
    print("⚠️  WARNING: Low distance, but continuing...")
    sys.exit(0)
else:
    print("✅ Data quality OK")
    sys.exit(0)
EOF

echo ""

# STEP 2: Preprocess
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

# STEP 3: Extract Features
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

# STEP 4: Train LSTM
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}STEP 4: Training LSTM${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
EPOCHS=${1:-30}
if [ ! -f "$DATA_DIR/lstm_model/best_model.pth" ]; then
    python3 training/train_lstm.py "$DATA_DIR" \
        --output "$DATA_DIR/lstm_model" \
        --epochs "$EPOCHS" \
        --batch-size 64
    echo -e "${GREEN}✅ Training complete${NC}"
else
    echo -e "${GREEN}✅ Model already trained${NC}"
fi
echo ""

# STEP 5: Update Config
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}STEP 5: Updating Configuration${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
MODEL_PATH="$DATA_DIR/lstm_model/best_model.pth"
if [ -f "$MODEL_PATH" ]; then
    python3 << PYEOF
import yaml

with open('config.yaml', 'r') as f:
    config = yaml.safe_load(f)

config['temporal']['trained_model_path'] = '$MODEL_PATH'

with open('config.yaml', 'w') as f:
    yaml.dump(config, f, default_flow_style=False, sort_keys=False)

print("✅ Config updated with trained model")
PYEOF
    echo -e "${GREEN}✅ Configuration updated${NC}"
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
echo ""

