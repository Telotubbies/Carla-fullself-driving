#!/bin/bash
# Complete Auto Pipeline: ตั้งแต่ต้นจนจบ - ระบบดีๆ
# STEP 1: Collect Data (ถ้ายังไม่มี)
# STEP 2: Create Lane Labels จาก CARLA
# STEP 3: Fine-tune ResNet สำหรับ Lane Detection (ปรับปรุงแล้ว)
# STEP 4: Extract Features ด้วย Fine-tuned ResNet
# STEP 5: Train LSTM
# STEP 6: Update Config
# STEP 7: Run Inference with GUI

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
echo -e "${BLUE}🚀 Complete Auto Pipeline (ระบบดีๆ)${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo ""

# Find latest data directory or create new one
LATEST_DATA=$(ls -td data/autopilot_* 2>/dev/null | head -1)

# STEP 1: Check if we need to collect data
if [ -z "$LATEST_DATA" ] || [ ! -f "$LATEST_DATA/data.csv" ] || [ $(wc -l < "$LATEST_DATA/data.csv" 2>/dev/null || echo 0) -lt 1000 ]; then
    echo -e "${YELLOW}════════════════════════════════════════${NC}"
    echo -e "${YELLOW}STEP 1: Collecting Data${NC}"
    echo -e "${YELLOW}════════════════════════════════════════${NC}"
    echo -e "${YELLOW}This may take a while (50,000 frames for better training)...${NC}"
    echo ""
    
    # Ensure CARLA is running
    if ! pgrep -f "CarlaUE4" > /dev/null; then
        echo -e "${YELLOW}⚠️  CARLA not running. Starting CARLA...${NC}"
        cd "$CARLA_DIR"
        ./CarlaUE4.sh -quality-level=Low -prefernoloadscreen > /tmp/carla.log 2>&1 &
        echo -e "${YELLOW}⏳ Waiting 40 seconds for CARLA to load...${NC}"
        sleep 40
        cd "$(dirname "$0")"
    fi
    
    # Collect more data: 50,000 frames for better training
    python3 training/collect_autopilot_data.py --frames 50000
    
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

# STEP 2: Create Lane Labels จาก CARLA
LANE_MASKS_DIR="$LATEST_DATA/lane_masks"
if [ ! -d "$LANE_MASKS_DIR" ] || [ $(find "$LANE_MASKS_DIR" -name "*_lane.png" 2>/dev/null | wc -l) -lt 100 ]; then
    echo -e "${YELLOW}════════════════════════════════════════${NC}"
    echo -e "${YELLOW}STEP 2: Creating Lane Labels from CARLA${NC}"
    echo -e "${YELLOW}════════════════════════════════════════${NC}"
    echo -e "${YELLOW}This will extract lane markings from CARLA map...${NC}"
    echo ""
    
    # Ensure CARLA is running
    if ! pgrep -f "CarlaUE4" > /dev/null; then
        echo -e "${YELLOW}⚠️  CARLA not running. Starting CARLA...${NC}"
        cd "$CARLA_DIR"
        ./CarlaUE4.sh -quality-level=Low -prefernoloadscreen > /tmp/carla.log 2>&1 &
        echo -e "${YELLOW}⏳ Waiting 40 seconds for CARLA to load...${NC}"
        sleep 40
        cd "$(dirname "$0")"
    fi
    
    # Check if CARLA is running and has vehicle
    if ! timeout 5 python3 -c "import carla; c=carla.Client('localhost',2000); c.set_timeout(3); w=c.get_world(); len(w.get_actors().filter('vehicle.*'))" 2>/dev/null | grep -q "[1-9]"; then
        echo -e "${YELLOW}⚠️  No vehicle in CARLA. Lane labels will be skipped for now.${NC}"
        echo -e "${YELLOW}   You can create them later when CARLA has a vehicle.${NC}"
        echo -e "${YELLOW}   Run: python3 training/create_lane_labels.py --images-dir $LATEST_DATA/images --output-dir $LANE_MASKS_DIR${NC}"
    else
        python3 training/create_lane_labels.py \
            --images-dir "$LATEST_DATA/images" \
            --output-dir "$LANE_MASKS_DIR" \
            --carla-host localhost \
            --carla-port 2000
        
        if [ $? -ne 0 ]; then
            echo -e "${YELLOW}⚠️  Lane label creation failed, but continuing...${NC}"
        fi
    fi
    
    echo -e "${GREEN}✅ Lane labels created!${NC}"
    echo ""
else
    echo -e "${GREEN}✅ Lane labels already exist${NC}"
    echo ""
fi

# STEP 3: Fine-tune ResNet สำหรับ Lane Detection (ปรับปรุงแล้ว)
RESNET_LANE_MODEL="$LATEST_DATA/resnet_lane_model/resnet_lane_final.pth"
if [ ! -f "$RESNET_LANE_MODEL" ]; then
    # Check if lane masks exist
    if [ -d "$LANE_MASKS_DIR" ] && [ $(find "$LANE_MASKS_DIR" -name "*_lane.png" 2>/dev/null | wc -l) -ge 100 ]; then
        echo -e "${YELLOW}════════════════════════════════════════${NC}"
        echo -e "${YELLOW}STEP 3: Fine-tuning ResNet for Lane Detection${NC}"
        echo -e "${YELLOW}════════════════════════════════════════${NC}"
        echo -e "${YELLOW}This will train ResNet to detect lanes better...${NC}"
        echo -e "${YELLOW}Using: DiceLoss + BCE Loss, Validation, Data Augmentation${NC}"
        echo ""
        
        python3 training/finetune_resnet_lane.py \
            --data-dir "$LATEST_DATA" \
            --masks-dir "$LANE_MASKS_DIR" \
            --epochs 20 \
            --batch-size 16 \
            --lr 0.001
        
        if [ $? -ne 0 ]; then
            echo -e "${YELLOW}⚠️  ResNet fine-tuning failed, continuing without fine-tuned model...${NC}"
        else
            echo -e "${GREEN}✅ ResNet fine-tuning complete!${NC}"
        fi
        echo ""
    else
        echo -e "${YELLOW}⚠️  Lane masks not found, skipping ResNet fine-tuning${NC}"
        echo -e "${YELLOW}   Will use pretrained ResNet instead${NC}"
        echo ""
    fi
else
    echo -e "${GREEN}✅ Fine-tuned ResNet already exists${NC}"
    echo ""
fi

# STEP 4: Check if we need to extract features
if [ ! -f "$LATEST_DATA/features.npy" ]; then
    echo -e "${YELLOW}════════════════════════════════════════${NC}"
    echo -e "${YELLOW}STEP 4: Extracting Features${NC}"
    echo -e "${YELLOW}════════════════════════════════════════${NC}"
    echo -e "${YELLOW}Using fine-tuned ResNet for better lane features...${NC}"
    echo ""
    
    # Use fine-tuned ResNet if available
    if [ -f "$RESNET_LANE_MODEL" ]; then
        echo -e "${GREEN}Using fine-tuned ResNet: $RESNET_LANE_MODEL${NC}"
        python3 training/extract_features.py \
            --data-dir "$LATEST_DATA" \
            --preprocess \
            --resnet-model "$RESNET_LANE_MODEL"
    else
        python3 training/extract_features.py \
            --data-dir "$LATEST_DATA" \
            --preprocess
    fi
    
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

# STEP 5: Check if we need to train LSTM
MODEL_PATH="$LATEST_DATA/lstm_model/best_model.pth"
if [ ! -f "$MODEL_PATH" ]; then
    echo -e "${YELLOW}════════════════════════════════════════${NC}"
    echo -e "${YELLOW}STEP 5: Training LSTM Model${NC}"
    echo -e "${YELLOW}════════════════════════════════════════${NC}"
    echo -e "${YELLOW}This will take a while (150 epochs with advanced techniques)...${NC}"
    echo -e "${YELLOW}  - Attention LSTM${NC}"
    echo -e "${YELLOW}  - Advanced Loss (MSE + Huber + Regularization)${NC}"
    echo -e "${YELLOW}  - Gradient Clipping${NC}"
    echo -e "${YELLOW}  - Early Stopping${NC}"
    echo ""
    
    # Advanced training with more epochs and techniques
    python3 training/train_lstm.py "$LATEST_DATA" \
        --epochs 150 \
        --batch-size 64 \
        --lr 0.001 \
        --sequence-length 10 \
        --hidden-size 256 \
        --num-layers 2 \
        --use-attention \
        --use-advanced-loss \
        --gradient-clip 1.0 \
        --early-stopping 20
    
    if [ $? -ne 0 ]; then
        echo -e "${RED}❌ Training failed!${NC}"
        exit 1
    fi
    
    echo -e "${GREEN}✅ Training complete!${NC}"
    echo ""
else
    echo -e "${GREEN}✅ Trained model already exists${NC}"
    echo ""
fi

# STEP 6: Update config.yaml
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}STEP 6: Updating config.yaml${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
python3 << EOF
import yaml

with open('config.yaml', 'r') as f:
    config = yaml.safe_load(f)

config['temporal']['trained_model_path'] = '$MODEL_PATH'

# Update ResNet model path if fine-tuned
if '$RESNET_LANE_MODEL' and __import__('os').path.exists('$RESNET_LANE_MODEL'):
    config['perception']['model_path'] = '$RESNET_LANE_MODEL'
    config['perception']['freeze_backbone'] = False

with open('config.yaml', 'w') as f:
    yaml.dump(config, f, default_flow_style=False, sort_keys=False)

print("✅ Updated config.yaml")
print(f"   LSTM model: $MODEL_PATH")
if '$RESNET_LANE_MODEL' and __import__('os').path.exists('$RESNET_LANE_MODEL'):
    print(f"   ResNet model: $RESNET_LANE_MODEL")
EOF

echo -e "${GREEN}✅ Configuration updated${NC}"
echo ""

# STEP 7: Run Inference with GUI
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}STEP 7: Running Inference with GUI${NC}"
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
echo "  Lane masks: $LANE_MASKS_DIR"
echo "  Fine-tuned ResNet: $RESNET_LANE_MODEL"
echo "  Trained LSTM: $MODEL_PATH"
echo "  Config updated: config.yaml"
echo ""

