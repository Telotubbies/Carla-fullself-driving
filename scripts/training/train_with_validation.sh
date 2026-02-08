#!/bin/bash
# 🎯 Training Flow with Validation
# 
# Complete training pipeline with validation checks:
# 1. ResNet Fine-tuning (300 epochs)
# 2. Feature Extraction
# 3. LSTM Training (150 epochs)
# 4. Validation & Status Update

set -e

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$PROJECT_ROOT"

# Set ROCm environment
export HSA_OVERRIDE_GFX_VERSION=11.0.0
export AMD_SERIALIZE_KERNEL=3
export HIP_FORCE_DEV_KERNELS=1

# CARLA environment
export CARLA_DIR=/home/a/Desktop/CARLA_0.9.16
CARLA_EGG=$(find "$CARLA_DIR/PythonAPI/carla/dist" -name "carla-*.egg" 2>/dev/null | head -1)
if [ -n "$CARLA_EGG" ]; then
    export PYTHONPATH="$CARLA_EGG:$PYTHONPATH"
fi

# Logging
LOG_DIR="$PROJECT_ROOT/logs/training"
mkdir -p "$LOG_DIR"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
TRAIN_LOG="$LOG_DIR/train_flow_${TIMESTAMP}.log"

log() {
    echo -e "$1" | tee -a "$TRAIN_LOG"
}

log_header() {
    log ""
    log "${BLUE}════════════════════════════════════════════════════════${NC}"
    log "${BLUE}$1${NC}"
    log "${BLUE}════════════════════════════════════════════════════════${NC}"
    log ""
}

log_step() {
    log ""
    log "${CYAN}▶ $1${NC}"
    log ""
}

log_success() {
    log "${GREEN}✅ $1${NC}"
}

log_warning() {
    log "${YELLOW}⚠️  $1${NC}"
}

log_error() {
    log "${RED}❌ $1${NC}"
}

# Find latest data
LATEST_DATA=$(ls -td data/autopilot_* 2>/dev/null | head -1)

if [ -z "$LATEST_DATA" ]; then
    log_error "No data directory found!"
    log "Please run data collection first: ./auto_flow.sh --skip-training"
    exit 1
fi

log_header "🎯 Training Flow with Validation"
log "Data directory: $LATEST_DATA"
log "Log file: $TRAIN_LOG"
log "Start time: $(date)"
log ""

# STEP 1: ResNet Fine-tuning
log_header "🎯 STEP 1: Fine-tune ResNet (300 epochs)"

RESNET_MODEL="$LATEST_DATA/resnet_lane_model/resnet_lane_final.pth"
RESNET_BEST="$LATEST_DATA/resnet_lane_model/resnet_lane_best.pth"
LANE_MASKS_DIR="$LATEST_DATA/lane_masks"

# Check if already trained to 300 epochs
if [ -f "$RESNET_MODEL" ]; then
    # Check training history for epoch count
    HISTORY="$LATEST_DATA/resnet_lane_model/training_history.json"
    if [ -f "$HISTORY" ]; then
        EPOCHS_TRAINED=$(python3 << EOF
import json
try:
    with open('$HISTORY', 'r') as f:
        h = json.load(f)
    print(h.get('total_epochs', 0))
except:
    print(0)
EOF
)
        if [ "$EPOCHS_TRAINED" -ge 300 ]; then
            log_success "ResNet already trained to 300 epochs"
            log "   Best Val Loss: $(python3 << 'PYEOF'
import json
try:
    with open('$HISTORY', 'r') as f:
        h = json.load(f)
    print(f"{h.get('best_val_loss', 'N/A'):.4f}")
except:
    print("N/A")
PYEOF
)"
        else
            log_warning "ResNet trained to $EPOCHS_TRAINED/300 epochs. Continuing..."
        fi
    else
        log_warning "No training history found. Will train from scratch or continue."
    fi
fi

# Check lane masks
MASK_COUNT=$(find "$LANE_MASKS_DIR" -name "*.png" 2>/dev/null | wc -l)
if [ "$MASK_COUNT" -lt 1000 ]; then
    log_error "Insufficient lane masks: $MASK_COUNT (need at least 1000)"
    exit 1
fi

log_step "Training ResNet for lane detection..."
log "   Masks: $MASK_COUNT"
log "   Epochs: 300"
log "   Batch Size: 16"
log "   Learning Rate: 0.001"
log ""

OUTPUT_DIR="$LATEST_DATA/resnet_lane_model"
mkdir -p "$OUTPUT_DIR"

# Check if already trained to 300 epochs
HISTORY="$OUTPUT_DIR/training_history.json"
if [ -f "$HISTORY" ]; then
    EPOCHS_TRAINED=$(python3 << EOF
import json
try:
    with open('$HISTORY', 'r') as f:
        h = json.load(f)
    print(h.get('total_epochs', 0))
except:
    print(0)
EOF
)
    if [ "$EPOCHS_TRAINED" -ge 300 ]; then
        log_success "ResNet already trained to 300 epochs. Skipping..."
    else
        log "Continuing training from epoch $EPOCHS_TRAINED to 300..."
        python3 training/finetune_resnet_lane.py \
            --data-dir "$LATEST_DATA" \
            --masks-dir "$LANE_MASKS_DIR" \
            --output-dir "$OUTPUT_DIR" \
            --epochs 300 \
            --batch-size 16 \
            --lr 0.001 \
            2>&1 | tee -a "$TRAIN_LOG"
    fi
else
    log "Starting ResNet training from scratch..."
    python3 training/finetune_resnet_lane.py \
        --data-dir "$LATEST_DATA" \
        --masks-dir "$LANE_MASKS_DIR" \
        --output-dir "$OUTPUT_DIR" \
        --epochs 300 \
        --batch-size 16 \
        --lr 0.001 \
        2>&1 | tee -a "$TRAIN_LOG"
fi

if [ -f "$RESNET_MODEL" ]; then
    # Get final loss
    FINAL_LOSS=$(python3 << 'PYEOF'
import json
try:
    with open('$OUTPUT_DIR/training_history.json', 'r') as f:
        h = json.load(f)
    val_loss = h.get('best_val_loss', 'N/A')
    if isinstance(val_loss, (int, float)):
        print(f"{val_loss:.4f}")
    else:
        print("N/A")
except:
    print("N/A")
PYEOF
)
    log_success "ResNet training complete"
    log "   Final Val Loss: $FINAL_LOSS"
    
    if [ "$FINAL_LOSS" != "N/A" ]; then
        # Check if loss is acceptable
        if (( $(echo "$FINAL_LOSS < 0.5" | bc -l) )); then
            log_success "Loss is good! (< 0.5)"
        elif (( $(echo "$FINAL_LOSS < 0.7" | bc -l) )); then
            log_warning "Loss is acceptable (< 0.7)"
        else
            log_warning "Loss is still high (> 0.7). May need more training or data."
        fi
    fi
else
    log_error "ResNet training failed"
    exit 1
fi

# STEP 2: Extract Features
log_header "🔍 STEP 2: Extract Features"

if [ -f "$LATEST_DATA/features.npy" ]; then
    log_warning "Features already exist. Re-extracting with fine-tuned ResNet..."
fi

log_step "Extracting features with fine-tuned ResNet..."

python3 training/extract_features.py \
    --data-dir "$LATEST_DATA" \
    --resnet-model "$RESNET_MODEL" \
    2>&1 | tee -a "$TRAIN_LOG"

if [ -f "$LATEST_DATA/features.npy" ]; then
    FEATURE_SIZE=$(du -h "$LATEST_DATA/features.npy" | cut -f1)
    log_success "Feature extraction complete"
    log "   Feature size: $FEATURE_SIZE"
else
    log_error "Feature extraction failed"
    exit 1
fi

# STEP 3: Train LSTM
log_header "🧠 STEP 3: Train LSTM (150 epochs)"

LSTM_MODEL="$LATEST_DATA/lstm_model/best_model.pth"

# Check if already trained
if [ -f "$LSTM_MODEL" ]; then
    HISTORY="$LATEST_DATA/lstm_model/training_history.json"
    if [ -f "$HISTORY" ]; then
        EPOCHS_TRAINED=$(python3 << EOF
import json
try:
    with open('$HISTORY', 'r') as f:
        h = json.load(f)
    print(h.get('total_epochs', 0))
except:
    print(0)
EOF
)
        if [ "$EPOCHS_TRAINED" -ge 150 ]; then
            log_success "LSTM already trained to 150 epochs"
            log "   Best Val Loss: $(python3 << 'PYEOF'
import json
try:
    with open('$HISTORY', 'r') as f:
        h = json.load(f)
    val_loss = h.get('best_val_loss', 'N/A')
    if isinstance(val_loss, (int, float)):
        print(f"{val_loss:.4f}")
    else:
        print("N/A")
except:
    print("N/A")
PYEOF
)"
        else
            log_warning "LSTM trained to $EPOCHS_TRAINED/150 epochs. Continuing..."
        fi
    else
        log_warning "No training history found. Will train from scratch."
    fi
fi

log_step "Training LSTM..."
log "   Epochs: 150"
log "   Batch Size: 64"
log "   Features: $LATEST_DATA/features.npy"
log ""

OUTPUT_DIR="$LATEST_DATA/lstm_model"
mkdir -p "$OUTPUT_DIR"

python3 training/train_lstm.py \
    --data-dir "$LATEST_DATA" \
    --epochs 150 \
    --batch-size 64 \
    --use-attention \
    --use-advanced-loss \
    --gradient-clip 1.0 \
    --early-stopping 20 \
    2>&1 | tee -a "$TRAIN_LOG"

if [ -f "$LSTM_MODEL" ]; then
    # Get final loss
    FINAL_LOSS=$(python3 << 'PYEOF'
import json
try:
    with open('$OUTPUT_DIR/training_history.json', 'r') as f:
        h = json.load(f)
    val_loss = h.get('best_val_loss', 'N/A')
    if isinstance(val_loss, (int, float)):
        print(f"{val_loss:.4f}")
    else:
        print("N/A")
except:
    print("N/A")
PYEOF
)
    log_success "LSTM training complete"
    log "   Final Val Loss: $FINAL_LOSS"
else
    log_error "LSTM training failed"
    exit 1
fi

# STEP 4: Update Config
log_header "⚙️  STEP 4: Update Configuration"

python3 << EOF
import yaml
from pathlib import Path

config_path = Path("config.yaml")
with open(config_path, 'r') as f:
    config = yaml.safe_load(f)

# Update paths
config['temporal']['trained_model_path'] = "$LSTM_MODEL"
if Path("$RESNET_MODEL").exists():
    config['perception']['resnet_model_path'] = "$RESNET_MODEL"
    config['perception']['lane_detection_model_path'] = "$RESNET_MODEL"
    config['perception']['freeze_backbone'] = False

with open(config_path, 'w') as f:
    yaml.dump(config, f, default_flow_style=False, sort_keys=False)

print("✅ Config updated")
EOF

log_success "Configuration updated"

# STEP 5: Update Status
log_header "📊 STEP 5: Update Status"

python3 scripts/view_status.py >> "$TRAIN_LOG" 2>&1

log_success "Status updated"

# Summary
log_header "✅ Training Flow Complete"
log "End time: $(date)"
log ""
log "📊 Summary:"
log "   ResNet Model: $RESNET_MODEL"
log "   LSTM Model: $LSTM_MODEL"
log "   Features: $LATEST_DATA/features.npy"
log "   Config: config.yaml (updated)"
log ""
log "📝 Log saved to: $TRAIN_LOG"
log ""
log_success "Ready for inference!"

