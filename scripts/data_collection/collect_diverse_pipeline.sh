#!/bin/bash
# Collect diverse data pipeline with multiple spawn points and augmentation

set -e

export HSA_OVERRIDE_GFX_VERSION=11.0.0
export CARLA_DIR=/home/a/Desktop/CARLA_0.9.16
CARLA_EGG=$(find "$CARLA_DIR/PythonAPI/carla/dist" -name "carla-*.egg" 2>/dev/null | head -1)
export PYTHONPATH="$CARLA_EGG:$PYTHONPATH"

cd "$(dirname "$0")"

echo "🚀 Starting Diverse Data Collection Pipeline"
echo "=============================================="
echo ""

# Configuration
FRAMES=50000
SPAWN_POINTS=10
FRAMES_PER_SPAWN=5000

echo "📋 Configuration:"
echo "   - Total frames: $FRAMES"
echo "   - Spawn points: $SPAWN_POINTS"
echo "   - Frames per spawn: $FRAMES_PER_SPAWN"
echo ""

# Step 1: Collect diverse data
echo "Step 1: Collecting diverse data..."
python3 training/collect_diverse_data.py \
    --frames $FRAMES \
    --spawn-points $SPAWN_POINTS \
    --frames-per-spawn $FRAMES_PER_SPAWN

if [ $? -ne 0 ]; then
    echo "❌ Data collection failed"
    exit 1
fi

# Find the latest data directory
LATEST_DATA_DIR=$(ls -td data/diverse_* 2>/dev/null | head -1)

if [ -z "$LATEST_DATA_DIR" ]; then
    echo "❌ No data directory found"
    exit 1
fi

echo ""
echo "✅ Data collection complete: $LATEST_DATA_DIR"
echo ""

# Step 2: Augment data
echo "Step 2: Augmenting data for steering diversity..."
python3 training/data_augmentation.py \
    --data-dir "$LATEST_DATA_DIR" \
    --factor 2.0 \
    --balance

if [ $? -ne 0 ]; then
    echo "❌ Data augmentation failed"
    exit 1
fi

AUGMENTED_DIR="${LATEST_DATA_DIR}_augmented"
echo ""
echo "✅ Data augmentation complete: $AUGMENTED_DIR"
echo ""

# Step 3: Preprocess
echo "Step 3: Preprocessing data..."
python3 training/data_preprocessing.py \
    --data-dir "$AUGMENTED_DIR"

if [ $? -ne 0 ]; then
    echo "❌ Data preprocessing failed"
    exit 1
fi

echo ""
echo "✅ Preprocessing complete"
echo ""

# Step 4: Extract features
echo "Step 4: Extracting ResNet features..."
python3 training/extract_features.py \
    --data-dir "$AUGMENTED_DIR"

if [ $? -ne 0 ]; then
    echo "❌ Feature extraction failed"
    exit 1
fi

echo ""
echo "✅ Feature extraction complete"
echo ""

# Summary
echo "=============================================="
echo "✅ Diverse Data Collection Pipeline Complete!"
echo "=============================================="
echo ""
echo "📊 Output:"
echo "   - Original data: $LATEST_DATA_DIR"
echo "   - Augmented data: $AUGMENTED_DIR"
echo ""
echo "Next steps:"
echo "   1. Train LSTM: python3 training/train_lstm.py --data-dir $AUGMENTED_DIR"
echo "   2. Run inference: python3 main.py --mode inference"
echo ""

