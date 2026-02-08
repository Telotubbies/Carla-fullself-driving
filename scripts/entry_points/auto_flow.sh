#!/bin/bash
# 🚀 Automated Complete Flow - CARLA LSTM-MPC Pipeline
# 
# This script runs the complete pipeline automatically:
# 1. Data Collection
# 2. Lane Label Creation
# 3. ResNet Fine-tuning
# 4. Feature Extraction
# 5. LSTM Training
# 6. Inference with GUI
#
# Usage: ./auto_flow.sh [--skip-data] [--skip-training] [--inference-only]

set -e

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$PROJECT_ROOT"

# Parse arguments
SKIP_DATA=false
SKIP_TRAINING=false
INFERENCE_ONLY=false

for arg in "$@"; do
    case $arg in
        --skip-data)
            SKIP_DATA=true
            shift
            ;;
        --skip-training)
            SKIP_TRAINING=true
            shift
            ;;
        --inference-only)
            INFERENCE_ONLY=true
            shift
            ;;
        *)
            ;;
    esac
done

# Set ROCm environment (for AMD GPU)
export HSA_OVERRIDE_GFX_VERSION=11.0.0
export AMD_SERIALIZE_KERNEL=3
export HIP_FORCE_DEV_KERNELS=1

# CARLA environment
export CARLA_DIR=/home/a/Desktop/CARLA_0.9.16
CARLA_EGG=$(find "$CARLA_DIR/PythonAPI/carla/dist" -name "carla-*.egg" 2>/dev/null | head -1)
if [ -n "$CARLA_EGG" ]; then
    export PYTHONPATH="$CARLA_EGG:$PYTHONPATH"
fi

# Logging - Enhanced logging system
LOG_DIR="$PROJECT_ROOT/logs"
FLOW_LOG_DIR="$LOG_DIR/flow"
mkdir -p "$FLOW_LOG_DIR"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
FLOW_LOG="$FLOW_LOG_DIR/auto_flow_${TIMESTAMP}.log"
STEP_LOG_DIR="$FLOW_LOG_DIR/steps_${TIMESTAMP}"
mkdir -p "$STEP_LOG_DIR"

# Create summary log
SUMMARY_LOG="$FLOW_LOG_DIR/summary_${TIMESTAMP}.log"

log() {
    echo -e "$1" | tee -a "$FLOW_LOG"
}

log_to_file() {
    local log_file="$1"
    shift
    echo -e "$@" >> "$log_file"
}

log_step_file() {
    local step_name="$1"
    local step_log="$STEP_LOG_DIR/${step_name}.log"
    {
        echo "Step: $step_name"
        echo "Start: $(date)"
        echo "========================================"
    } > "$step_log"
    echo "$step_log"
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

# Check prerequisites
check_prerequisites() {
    log_header "🔍 Checking Prerequisites"
    
    # Check Python
    if ! command -v python3 &> /dev/null; then
        log_error "Python3 not found!"
        exit 1
    fi
    log_success "Python3: $(python3 --version)"
    
    # Check CARLA
    if ! pgrep -f "CarlaUE4" > /dev/null; then
        log_warning "CARLA not running. Will start automatically."
    else
        log_success "CARLA is running"
    fi
    
    # Check dependencies
    if ! python3 -c "import torch" 2>/dev/null; then
        log_error "PyTorch not installed!"
        exit 1
    fi
    log_success "PyTorch installed"
    
    if ! python3 -c "import casadi" 2>/dev/null; then
        log_error "CasADi not installed!"
        exit 1
    fi
    log_success "CasADi installed"
}

# Start CARLA if needed
start_carla() {
    if pgrep -f "CarlaUE4" > /dev/null; then
        log_success "CARLA already running"
        return 0
    fi
    
    log_step "Starting CARLA..."
    cd "$CARLA_DIR"
    ./CarlaUE4.sh -quality-level=Low -RenderOffScreen > /dev/null 2>&1 &
    CARLA_PID=$!
    cd "$PROJECT_ROOT"
    
    log "Waiting for CARLA to start..."
    sleep 10
    
    # Wait for CARLA to be ready
    for i in {1..30}; do
        if python3 -c "import carla; client = carla.Client('localhost', 2000); client.get_world()" 2>/dev/null; then
            log_success "CARLA is ready"
            return 0
        fi
        sleep 2
    done
    
    log_error "CARLA failed to start"
    return 1
}

# STEP 1: Data Collection
step_collect_data() {
    log_header "📊 STEP 1: Data Collection"
    
    LATEST_DATA=$(ls -td data/autopilot_* 2>/dev/null | head -1)
    
    if [ -n "$LATEST_DATA" ] && [ -f "$LATEST_DATA/data.csv" ]; then
        FRAME_COUNT=$(wc -l < "$LATEST_DATA/data.csv" 2>/dev/null || echo 0)
        if [ "$FRAME_COUNT" -ge 10000 ]; then
            log_success "Data already exists: $LATEST_DATA ($FRAME_COUNT frames)"
            return 0
        fi
    fi
    
    log_step "Collecting 50,000 frames from CARLA autopilot..."
    
    if ! start_carla; then
        log_error "Failed to start CARLA"
        return 1
    fi
    
    STEP_LOG=$(log_step_file "01_data_collection")
    python3 training/collect_autopilot_data.py \
        --frames 50000 \
        --output-dir "data" \
        2>&1 | tee -a "$FLOW_LOG" | tee -a "$STEP_LOG"
    
    echo "End: $(date)" >> "$STEP_LOG"
    echo "Status: $([ $? -eq 0 ] && echo 'SUCCESS' || echo 'FAILED')" >> "$STEP_LOG"
    
    if [ $? -eq 0 ]; then
        LATEST_DATA=$(ls -td data/autopilot_* 2>/dev/null | head -1)
        log_success "Data collection complete: $LATEST_DATA"
        return 0
    else
        log_error "Data collection failed"
        return 1
    fi
}

# STEP 2: Create Lane Labels
step_create_lane_labels() {
    log_header "🛣️  STEP 2: Create Lane Labels"
    
    LATEST_DATA=$(ls -td data/autopilot_* 2>/dev/null | head -1)
    if [ -z "$LATEST_DATA" ]; then
        log_error "No data directory found"
        return 1
    fi
    
    IMAGES_DIR="$LATEST_DATA/images"
    LANE_MASKS_DIR="$LATEST_DATA/lane_masks"
    
    if [ -d "$LANE_MASKS_DIR" ] && [ $(find "$LANE_MASKS_DIR" -name "*.png" 2>/dev/null | wc -l) -ge 10000 ]; then
        log_success "Lane masks already exist ($(find "$LANE_MASKS_DIR" -name "*.png" 2>/dev/null | wc -l) masks)"
        return 0
    fi
    
    log_step "Creating lane labels from CARLA..."
    
    if ! start_carla; then
        log_error "Failed to start CARLA"
        return 1
    fi
    
    mkdir -p "$LANE_MASKS_DIR"
    
    STEP_LOG=$(log_step_file "02_create_lane_labels")
    
    # Try CARLA-based labels first
    python3 training/create_lane_labels.py \
        --images-dir "$IMAGES_DIR" \
        --output-dir "$LANE_MASKS_DIR" \
        --carla-host localhost \
        --carla-port 2000 \
        2>&1 | tee -a "$FLOW_LOG" | tee -a "$STEP_LOG"
    
    # Fallback to image processing if CARLA fails
    if [ $? -ne 0 ] || [ $(find "$LANE_MASKS_DIR" -name "*.png" 2>/dev/null | wc -l) -lt 1000 ]; then
        log_warning "CARLA labels failed, using image processing fallback..."
        python3 training/create_lane_masks_from_images.py \
            --images-dir "$IMAGES_DIR" \
            --output-dir "$LANE_MASKS_DIR" \
            --max-images 20000 \
            2>&1 | tee -a "$FLOW_LOG" | tee -a "$STEP_LOG"
    fi
    
    echo "End: $(date)" >> "$STEP_LOG"
    echo "Status: $([ $? -eq 0 ] && echo 'SUCCESS' || echo 'FAILED')" >> "$STEP_LOG"
    
    MASK_COUNT=$(find "$LANE_MASKS_DIR" -name "*.png" 2>/dev/null | wc -l)
    if [ "$MASK_COUNT" -ge 1000 ]; then
        log_success "Lane labels created: $MASK_COUNT masks"
        return 0
    else
        log_error "Failed to create lane labels"
        return 1
    fi
}

# STEP 3: Fine-tune ResNet
step_finetune_resnet() {
    log_header "🎯 STEP 3: Fine-tune ResNet for Lane Detection"
    
    LATEST_DATA=$(ls -td data/autopilot_* 2>/dev/null | head -1)
    if [ -z "$LATEST_DATA" ]; then
        log_error "No data directory found"
        return 1
    fi
    
    LANE_MASKS_DIR="$LATEST_DATA/lane_masks"
    OUTPUT_DIR="$LATEST_DATA/resnet_lane_model"
    
    if [ -f "$OUTPUT_DIR/resnet_lane_final.pth" ]; then
        log_success "ResNet model already exists: $OUTPUT_DIR/resnet_lane_final.pth"
        return 0
    fi
    
    MASK_COUNT=$(find "$LANE_MASKS_DIR" -name "*.png" 2>/dev/null | wc -l)
    if [ "$MASK_COUNT" -lt 1000 ]; then
        log_error "Insufficient lane masks: $MASK_COUNT (need at least 1000)"
        return 1
    fi
    
    log_step "Fine-tuning ResNet (300 epochs)..."
    log "   Masks: $MASK_COUNT"
    log "   Epochs: 300"
    log "   Batch Size: 16"
    log "   Learning Rate: 0.001"
    
    mkdir -p "$OUTPUT_DIR"
    
    STEP_LOG=$(log_step_file "03_finetune_resnet")
    python3 training/finetune_resnet_lane.py \
        --data-dir "$LATEST_DATA" \
        --masks-dir "$LANE_MASKS_DIR" \
        --output-dir "$OUTPUT_DIR" \
        --epochs 300 \
        --batch-size 16 \
        --lr 0.001 \
        2>&1 | tee -a "$FLOW_LOG" | tee -a "$STEP_LOG"
    
    echo "End: $(date)" >> "$STEP_LOG"
    echo "Status: $([ $? -eq 0 ] && echo 'SUCCESS' || echo 'FAILED')" >> "$STEP_LOG"
    
    if [ -f "$OUTPUT_DIR/resnet_lane_final.pth" ]; then
        log_success "ResNet fine-tuning complete"
        return 0
    else
        log_error "ResNet fine-tuning failed"
        return 1
    fi
}

# STEP 4: Extract Features
step_extract_features() {
    log_header "🔍 STEP 4: Extract Features"
    
    LATEST_DATA=$(ls -td data/autopilot_* 2>/dev/null | head -1)
    if [ -z "$LATEST_DATA" ]; then
        log_error "No data directory found"
        return 1
    fi
    
    if [ -f "$LATEST_DATA/features.npy" ]; then
        log_success "Features already extracted"
        return 0
    fi
    
    log_step "Extracting features with ResNet..."
    
    RESNET_MODEL="$LATEST_DATA/resnet_lane_model/resnet_lane_final.pth"
    STEP_LOG=$(log_step_file "04_extract_features")
    
    if [ -f "$RESNET_MODEL" ]; then
        log "Using fine-tuned ResNet: $RESNET_MODEL"
        python3 training/extract_features.py \
            --data-dir "$LATEST_DATA" \
            --resnet-model "$RESNET_MODEL" \
            2>&1 | tee -a "$FLOW_LOG" | tee -a "$STEP_LOG"
    else
        log_warning "Fine-tuned ResNet not found, using pretrained"
        python3 training/extract_features.py \
            --data-dir "$LATEST_DATA" \
            2>&1 | tee -a "$FLOW_LOG" | tee -a "$STEP_LOG"
    fi
    
    echo "End: $(date)" >> "$STEP_LOG"
    echo "Status: $([ $? -eq 0 ] && echo 'SUCCESS' || echo 'FAILED')" >> "$STEP_LOG"
    
    if [ -f "$LATEST_DATA/features.npy" ]; then
        log_success "Feature extraction complete"
        return 0
    else
        log_error "Feature extraction failed"
        return 1
    fi
}

# STEP 5: Train LSTM
step_train_lstm() {
    log_header "🧠 STEP 5: Train LSTM"
    
    LATEST_DATA=$(ls -td data/autopilot_* 2>/dev/null | head -1)
    if [ -z "$LATEST_DATA" ]; then
        log_error "No data directory found"
        return 1
    fi
    
    LSTM_MODEL="$LATEST_DATA/lstm_model/best_model.pth"
    if [ -f "$LSTM_MODEL" ]; then
        log_success "LSTM model already exists: $LSTM_MODEL"
        return 0
    fi
    
    if [ ! -f "$LATEST_DATA/features.npy" ]; then
        log_error "Features not found. Run feature extraction first."
        return 1
    fi
    
    log_step "Training LSTM (150 epochs)..."
    log "   Epochs: 150"
    log "   Batch Size: 64"
    log "   Features: $LATEST_DATA/features.npy"
    
    STEP_LOG=$(log_step_file "05_train_lstm")
    python3 training/train_lstm.py \
        --data-dir "$LATEST_DATA" \
        --epochs 150 \
        --batch-size 64 \
        --use-attention \
        --use-advanced-loss \
        --gradient-clip 1.0 \
        --early-stopping 20 \
        2>&1 | tee -a "$FLOW_LOG" | tee -a "$STEP_LOG"
    
    echo "End: $(date)" >> "$STEP_LOG"
    echo "Status: $([ $? -eq 0 ] && echo 'SUCCESS' || echo 'FAILED')" >> "$STEP_LOG"
    
    if [ -f "$LSTM_MODEL" ]; then
        log_success "LSTM training complete"
        return 0
    else
        log_error "LSTM training failed"
        return 1
    fi
}

# STEP 6: Update Config
step_update_config() {
    log_header "⚙️  STEP 6: Update Configuration"
    
    LATEST_DATA=$(ls -td data/autopilot_* 2>/dev/null | head -1)
    if [ -z "$LATEST_DATA" ]; then
        log_error "No data directory found"
        return 1
    fi
    
    LSTM_MODEL="$LATEST_DATA/lstm_model/best_model.pth"
    RESNET_MODEL="$LATEST_DATA/resnet_lane_model/resnet_lane_final.pth"
    
    if [ ! -f "$LSTM_MODEL" ]; then
        log_error "LSTM model not found"
        return 1
    fi
    
    log_step "Updating config.yaml with model paths..."
    
    # Update config using Python
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
}

# STEP 7: Run Inference
step_run_inference() {
    log_header "🚗 STEP 7: Run Inference with GUI"
    
    if ! start_carla; then
        log_error "Failed to start CARLA"
        return 1
    fi
    
    log_step "Starting autonomous driving system..."
    log "   Press Ctrl+C to stop"
    log ""
    
    STEP_LOG=$(log_step_file "07_inference")
    python3 main.py --mode inference --config config.yaml 2>&1 | tee -a "$FLOW_LOG" | tee -a "$STEP_LOG"
    
    echo "End: $(date)" >> "$STEP_LOG"
    echo "Status: $([ $? -eq 0 ] && echo 'SUCCESS' || echo 'FAILED')" >> "$STEP_LOG"
}

# Main flow
main() {
    log_header "🚀 CARLA LSTM-MPC Automated Flow"
    log "Start time: $(date)"
    log "Log file: $FLOW_LOG"
    
    check_prerequisites
    
    if [ "$INFERENCE_ONLY" = true ]; then
        log_header "Running Inference Only"
        step_run_inference
        exit $?
    fi
    
    # Training pipeline
    if [ "$SKIP_TRAINING" = false ]; then
        if [ "$SKIP_DATA" = false ]; then
            step_collect_data || exit 1
        fi
        
        step_create_lane_labels || exit 1
        step_finetune_resnet || exit 1
        step_extract_features || exit 1
        step_train_lstm || exit 1
        step_update_config || exit 1
    fi
    
    # Inference
    step_run_inference
    
    log_header "✅ Flow Complete"
    log "End time: $(date)"
    log "Log saved to: $FLOW_LOG"
    log "Step logs: $STEP_LOG_DIR"
    
    # Create summary log
    {
        echo "========================================"
        echo "AUTO FLOW SUMMARY"
        echo "========================================"
        echo "Start: $(head -1 "$FLOW_LOG" | grep -o '[0-9].*' || echo 'N/A')"
        echo "End: $(date)"
        echo "Main Log: $FLOW_LOG"
        echo "Step Logs: $STEP_LOG_DIR"
        echo ""
        echo "Steps Completed:"
        for step_log in "$STEP_LOG_DIR"/*.log; do
            if [ -f "$step_log" ]; then
                step_name=$(basename "$step_log" .log)
                step_status=$(grep "Status:" "$step_log" | tail -1 | awk '{print $2}')
                echo "  $step_name: $step_status"
            fi
        done
        echo ""
        echo "========================================"
    } > "$SUMMARY_LOG"
    
    log "Summary log: $SUMMARY_LOG"
}

# Run main
main "$@"

