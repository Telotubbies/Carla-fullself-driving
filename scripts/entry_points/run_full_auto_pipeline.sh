#!/bin/bash
# Full Automated Pipeline: Data Collection → Training → Inference

set -e

export HSA_OVERRIDE_GFX_VERSION=11.0.0
export CARLA_DIR=/home/a/Desktop/CARLA_0.9.16
CARLA_EGG=$(find "$CARLA_DIR/PythonAPI/carla/dist" -name "carla-*.egg" 2>/dev/null | head -1)
export PYTHONPATH="$CARLA_EGG:$PYTHONPATH"

cd "$(dirname "$0")"

echo "🚀 Full Automated Pipeline"
echo "=========================="
echo ""
echo "This will run:"
echo "  1. Data Collection (20,000 frames)"
echo "  2. Data Preprocessing"
echo "  3. Feature Extraction (ResNet)"
echo "  4. LSTM Training"
echo "  5. (Optional) Lane Detection Training"
echo "  6. Update Config"
echo "  7. Run Inference with GUI"
echo ""

# Configuration
FRAMES=${1:-20000}
DATA_DIR="data"

# Step 1: Check CARLA
echo "Step 1: Checking CARLA..."
if ! timeout 5 python3 -c "import carla; c=carla.Client('localhost',2000); c.set_timeout(3); c.get_world()" 2>/dev/null; then
    echo "⚠️  CARLA not ready, starting..."
    cd "$CARLA_DIR"
    ./CarlaUE4.sh -quality-level=Low -prefernoloadscreen > /tmp/carla.log 2>&1 &
    echo "⏳ Waiting 40 seconds for CARLA to load..."
    sleep 40
    
    # Check again
    cd "$(dirname "$0")"
    if ! timeout 10 python3 -c "import carla; c=carla.Client('localhost',2000); c.set_timeout(5); c.get_world()" 2>/dev/null; then
        echo "❌ CARLA failed to start"
        exit 1
    fi
fi
echo "✅ CARLA is ready"
echo ""

# Step 2: Data Collection
echo "Step 2: Data Collection ($FRAMES frames)..."
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTPUT_DIR="$DATA_DIR/autopilot_$TIMESTAMP"

python3 training/collect_autopilot_data.py --frames $FRAMES --output "$OUTPUT_DIR" > logs/collection_$TIMESTAMP.log 2>&1

if [ $? -ne 0 ]; then
    echo "❌ Data collection failed"
    exit 1
fi

echo "✅ Data collection complete: $OUTPUT_DIR"
echo ""

# Step 3: Data Preprocessing
echo "Step 3: Data Preprocessing..."
python3 -c "
from training.data_preprocessing import preprocess_dataset
import sys
preprocess_dataset('$OUTPUT_DIR')
" >> logs/collection_$TIMESTAMP.log 2>&1

if [ $? -ne 0 ]; then
    echo "⚠️  Preprocessing failed, continuing..."
fi
echo "✅ Preprocessing complete"
echo ""

# Step 4: Feature Extraction
echo "Step 4: Extracting ResNet features..."
python3 training/extract_features.py --data-dir "$OUTPUT_DIR" >> logs/collection_$TIMESTAMP.log 2>&1

if [ $? -ne 0 ]; then
    echo "❌ Feature extraction failed"
    exit 1
fi
echo "✅ Feature extraction complete"
echo ""

# Step 5: Train LSTM
echo "Step 5: Training LSTM..."
python3 training/train_lstm.py \
    --data-dir "$OUTPUT_DIR" \
    --epochs 50 \
    --batch-size 32 \
    --lr 0.001 \
    > logs/training_$TIMESTAMP.log 2>&1

if [ $? -ne 0 ]; then
    echo "❌ LSTM training failed"
    exit 1
fi
echo "✅ LSTM training complete"
echo ""

# Step 6: Update Config
echo "Step 6: Updating config..."
MODEL_PATH="$OUTPUT_DIR/lstm_model/best_model.pth"
if [ -f "$MODEL_PATH" ]; then
    python3 -c "
import yaml
with open('config.yaml', 'r') as f:
    config = yaml.safe_load(f)
config['temporal']['trained_model_path'] = '$MODEL_PATH'
with open('config.yaml', 'w') as f:
    yaml.dump(config, f, default_flow_style=False)
print('✅ Config updated')
"
else
    echo "⚠️  Model not found, skipping config update"
fi
echo ""

# Step 7: (Optional) Lane Detection Training
read -p "Train Lane Detection? (y/n): " -t 5 TRAIN_LANE || TRAIN_LANE="n"
if [ "$TRAIN_LANE" = "y" ]; then
    echo "Step 7: Training Lane Detection..."
    ./scripts/training/train_lane_detection.sh "$OUTPUT_DIR" >> logs/lane_training_$TIMESTAMP.log 2>&1
    echo "✅ Lane detection training complete"
    echo ""
fi

# Step 8: Run Inference
echo "Step 8: Starting Inference with GUI..."
echo "   Press Ctrl+C to stop"
echo ""

python3 main.py --mode inference

echo ""
echo "✅ Full pipeline complete!"
echo ""
echo "📁 Outputs:"
echo "   Data: $OUTPUT_DIR"
echo "   Model: $MODEL_PATH"
echo "   Logs: logs/collection_$TIMESTAMP.log"
echo "   Training: logs/training_$TIMESTAMP.log"

