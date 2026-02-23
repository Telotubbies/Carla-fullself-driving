#!/bin/bash
# Download Ultra-Fast-Lane-Detection-v2 Pre-trained Models

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

# Default values
DATASET=${1:-culane}
BACKBONE=${2:-res18}
OUTPUT_DIR=${3:-weights}

echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}Download Ultra-Fast-Lane-Detection-v2${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo ""
echo -e "Dataset: ${GREEN}$DATASET${NC}"
echo -e "Backbone: ${GREEN}$BACKBONE${NC}"
echo -e "Output: ${GREEN}$OUTPUT_DIR${NC}"
echo ""

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Model information
MODEL_NAME="ufldv2_${DATASET}_${BACKBONE}.pth"
MODEL_PATH="$OUTPUT_DIR/$MODEL_NAME"

# Check if model already exists
if [ -f "$MODEL_PATH" ]; then
    echo -e "${GREEN}✅ Model already exists: $MODEL_PATH${NC}"
    exit 0
fi

echo -e "${YELLOW}⚠️  Manual Download Required${NC}"
echo ""
echo "Ultra-Fast-Lane-Detection-v2 models are hosted on Google Drive."
echo "Please download manually from:"
echo ""
echo -e "${BLUE}Repository:${NC} https://github.com/cfzd/Ultra-Fast-Lane-Detection-v2"
echo ""
echo -e "${BLUE}Available Models:${NC}"

case "$DATASET" in
    culane)
        if [ "$BACKBONE" = "res18" ]; then
            echo "  - ResNet18: https://drive.google.com/file/d/1oEjJraFr-3lxhX_OXduAGFWalWa6Xh3W/view?usp=sharing"
            echo "  - F1 Score: 75.0"
        elif [ "$BACKBONE" = "res34" ]; then
            echo "  - ResNet34: https://drive.google.com/file/d/1AjnvAD3qmqt_dGPveZJsLZ1bOyWv62Yj/view?usp=sharing"
            echo "  - F1 Score: 76.0"
        fi
        ;;
    tusimple)
        if [ "$BACKBONE" = "res18" ]; then
            echo "  - ResNet18: https://drive.google.com/file/d/1Clnj9-dLz81S3wXiYtlkc4HVusCb978t/view?usp=sharing"
            echo "  - Accuracy: 96.11"
        elif [ "$BACKBONE" = "res34" ]; then
            echo "  - ResNet34: https://drive.google.com/file/d/1pkz8homK433z39uStGK3ZWkDXrnBAMmX/view?usp=sharing"
            echo "  - Accuracy: 96.24"
        fi
        ;;
    curvelanes)
        if [ "$BACKBONE" = "res18" ]; then
            echo "  - ResNet18: https://drive.google.com/file/d/1VfbUvorKKMG4tUePNbLYPp63axgd-8BX/view?usp=sharing"
            echo "  - F1 Score: 80.42"
        elif [ "$BACKBONE" = "res34" ]; then
            echo "  - ResNet34: https://drive.google.com/file/d/1O1kPSr85Icl2JbwV3RBlxWZYhLEHo8EN/view?usp=sharing"
            echo "  - F1 Score: 81.34"
        fi
        ;;
    *)
        echo -e "${RED}❌ Unknown dataset: $DATASET${NC}"
        echo "Available: culane, tusimple, curvelanes"
        exit 1
        ;;
esac

echo ""
echo -e "${YELLOW}Instructions:${NC}"
echo "1. Click the link above"
echo "2. Download the .pth file"
echo "3. Save it as: $MODEL_PATH"
echo ""
echo -e "${BLUE}After downloading, verify with:${NC}"
echo "  ls -lh $MODEL_PATH"
echo ""
echo -e "${GREEN}Then update config.yaml:${NC}"
echo "  perception:"
echo "    lane_detection_model_path: \"$MODEL_PATH\""
echo "    lane_detection_model_type: \"ultra_fast\""

