#!/bin/bash
# Train Lane Detection Models

set -e

export HSA_OVERRIDE_GFX_VERSION=11.0.0
export CARLA_DIR=/home/a/Desktop/CARLA_0.9.16
CARLA_EGG=$(find "$CARLA_DIR/PythonAPI/carla/dist" -name "carla-*.egg" 2>/dev/null | head -1)
export PYTHONPATH="$CARLA_EGG:$PYTHONPATH"

cd "$(dirname "$0")/../.."

echo "🚀 Training Lane Detection Models"
echo "=================================="
echo ""

DATA_DIR="${1:-data/autopilot_20260208_150902}"

if [ ! -d "$DATA_DIR" ]; then
    echo "❌ Data directory not found: $DATA_DIR"
    exit 1
fi

IMAGES_DIR="$DATA_DIR/images"
MASKS_DIR="$DATA_DIR/lane_masks"

echo "📋 Configuration:"
echo "   Data dir: $DATA_DIR"
echo "   Images: $IMAGES_DIR"
echo ""

# Step 1: Create lane labels from CARLA
echo "Step 1: Creating lane labels from CARLA..."
echo "   (This requires CARLA to be running)"
echo "   python3 -c \"from perception.lane_detector import create_lane_labels_from_carla; ...\""
echo "   ⚠️  Manual step: Run lane labeling script with CARLA connection"
echo ""

# Step 2: Train U-Net
echo "Step 2: Training U-Net for lane detection..."
if [ -d "$MASKS_DIR" ] && [ "$(ls -A $MASKS_DIR 2>/dev/null)" ]; then
    python3 training/train_lane_unet.py \
        --images-dir "$IMAGES_DIR" \
        --masks-dir "$MASKS_DIR" \
        --epochs 20 \
        --batch-size 8 \
        --lr 0.0001
    echo "✅ U-Net training complete"
else
    echo "⚠️  Skipping U-Net training (no masks found)"
    echo "   Create masks first using CARLA lane detection"
fi

echo ""

# Step 3: Fine-tune ResNet
echo "Step 3: Fine-tuning ResNet for lane detection..."
python3 training/finetune_resnet_lane.py \
    --data-dir "$DATA_DIR" \
    --epochs 10 \
    --batch-size 16 \
    --lr 0.001 \
    --freeze

echo "✅ ResNet fine-tuning complete"
echo ""

echo "✅ Lane detection training complete!"
echo ""
echo "📁 Output models:"
echo "   - U-Net: $DATA_DIR/lane_unet_model/lane_unet_final.pth"
echo "   - ResNet: $DATA_DIR/resnet_lane_model/resnet_lane_final.pth"
echo ""

