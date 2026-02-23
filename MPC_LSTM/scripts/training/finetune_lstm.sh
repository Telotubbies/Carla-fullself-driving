#!/bin/bash
# Fine-tune LSTM Model Script

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
echo -e "${BLUE}🔄 Fine-tune LSTM Model${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo ""

# Default values
DATA_DIR="${1:-data/autopilot_20260208_150902}"
CHECKPOINT="${2:-data/autopilot_20260208_150902/lstm_model/best_model.pth}"
EPOCHS="${3:-50}"
LR="${4:-0.0001}"

# Check if checkpoint exists
if [ ! -f "$CHECKPOINT" ]; then
    echo -e "${RED}❌ Checkpoint not found: $CHECKPOINT${NC}"
    exit 1
fi

# Check if data directory exists
if [ ! -d "$DATA_DIR" ]; then
    echo -e "${RED}❌ Data directory not found: $DATA_DIR${NC}"
    exit 1
fi

echo -e "${GREEN}📋 Configuration:${NC}"
echo "   Data directory: $DATA_DIR"
echo "   Checkpoint: $CHECKPOINT"
echo "   Epochs: $EPOCHS"
echo "   Learning rate: $LR (fine-tuning rate)"
echo ""

# Run fine-tuning
echo -e "${YELLOW}Starting fine-tuning...${NC}"
echo ""

python3 training/finetune_lstm.py \
    "$DATA_DIR" \
    --checkpoint "$CHECKPOINT" \
    --epochs "$EPOCHS" \
    --lr "$LR" \
    --batch-size 64 \
    --gradient-clip 1.0 \
    --early-stopping 15

if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}════════════════════════════════════════${NC}"
    echo -e "${GREEN}✅ Fine-tuning complete!${NC}"
    echo -e "${GREEN}════════════════════════════════════════${NC}"
    echo ""
    echo "📁 Output models:"
    echo "   - Best model: $DATA_DIR/lstm_model/best_model.pth"
    echo "   - Fine-tuned: $DATA_DIR/lstm_model/fine_tuned_model.pth"
    echo ""
    echo "💡 To use the fine-tuned model, update config.yaml:"
    echo "   temporal:"
    echo "     trained_model_path: \"$DATA_DIR/lstm_model/fine_tuned_model.pth\""
    echo ""
else
    echo ""
    echo -e "${RED}❌ Fine-tuning failed!${NC}"
    exit 1
fi

