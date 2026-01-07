#!/bin/bash
# Master Pipeline Script - รันทุก step: IL → RL → Test
# ใช้แนวจาก auto_train.sh: monitoring, auto-restart, checkpoint management

set -e  # Exit on error (but we'll handle errors gracefully)

# Configuration
CARLA_DIR="/home/a/Desktop/CARLA_0.9.16"
RL_AGENT_DIR="/home/a/Desktop/CARLA_0.9.16/RL_Agent"
CONFIG_FILE="config/phase1_accelerated_learning.yaml"
CARLA_PORT=2000
MAX_RESTARTS=100
CHECK_INTERVAL=60

# Step Configuration
IL_NUM_EPISODES=40
IL_TOWN="Town01_Opt"
IL_AGENT="Basic"
RL_TOTAL_TIMESTEPS=500000
EVAL_NUM_EPISODES=10
DASHBOARD_PORT=5000

# Logging
LOG_DIR="$RL_AGENT_DIR/logs"
MASTER_LOG="$LOG_DIR/master_pipeline_$(date +%Y%m%d_%H%M%S).log"
PID_FILE="$RL_AGENT_DIR/.master_pipeline.pid"
STATE_FILE="$RL_AGENT_DIR/.master_pipeline_state"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# Function: Log with timestamp
log() {
    echo -e "${CYAN}[$(date '+%Y-%m-%d %H:%M:%S')]${NC} $1" | tee -a "$MASTER_LOG"
}

log_success() {
    echo -e "${GREEN}✅ $1${NC}" | tee -a "$MASTER_LOG"
}

log_error() {
    echo -e "${RED}❌ $1${NC}" | tee -a "$MASTER_LOG"
}

log_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}" | tee -a "$MASTER_LOG"
}

log_step() {
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}" | tee -a "$MASTER_LOG"
    echo -e "${BLUE}📋 $1${NC}" | tee -a "$MASTER_LOG"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}" | tee -a "$MASTER_LOG"
}

# Function: Save state
save_state() {
    echo "$1" > "$STATE_FILE"
}

# Function: Load state
load_state() {
    if [ -f "$STATE_FILE" ]; then
        cat "$STATE_FILE"
    else
        echo "step1"
    fi
}

# Function: Check if CARLA is running
check_carla() {
    python3 -c "
import carla
try:
    client = carla.Client('localhost', $CARLA_PORT)
    client.set_timeout(2.0)
    world = client.get_world()
    print('OK')
except:
    print('NOT_RUNNING')
" 2>/dev/null
}

# Function: Cleanup CARLA processes (improved - handles child processes and prevents freeze)
cleanup_carla_processes() {
    log_warning "🧹 Cleaning up old CARLA processes..."
    
    # First, try to close any Python CARLA connections to prevent hanging
    # Note: CARLA client doesn't have disconnect(), but we can set timeout and let it close
    python3 -c "
import carla
try:
    client = carla.Client('localhost', $CARLA_PORT)
    client.set_timeout(0.5)
    # Just accessing world and closing connection naturally
    world = client.get_world()
    del world
    del client
except:
    pass
" 2>/dev/null || true
    
    # Get all CARLA PIDs including children
    local carla_pids=$(pgrep -f "CarlaUE4" 2>/dev/null || true)
    
    if [ -z "$carla_pids" ]; then
        log_success "No CARLA processes to cleanup"
        return 0
    fi
    
    # Kill parent processes first (graceful)
    for pid in $carla_pids; do
        # Get process tree and kill children first
        local children=$(pgrep -P $pid 2>/dev/null || true)
        if [ -n "$children" ]; then
            for child in $children; do
                kill -TERM $child 2>/dev/null || true
            done
            sleep 1
        fi
        # Kill parent
        kill -TERM $pid 2>/dev/null || true
    done
    
    # Wait for processes to die (with timeout)
    local wait_count=0
    local max_wait=15
    while [ $wait_count -lt $max_wait ]; do
        if ! pgrep -f "CarlaUE4" > /dev/null; then
            log_success "All CARLA processes cleaned up"
            # Sync filesystem to prevent freeze
            sync 2>/dev/null || true
            return 0
        fi
        sleep 1
        wait_count=$((wait_count + 1))
    done
    
    # Force kill if still running (kill all processes in tree)
    if pgrep -f "CarlaUE4" > /dev/null; then
        log_warning "⚠️  Force killing remaining CARLA processes..."
        
        # Get all PIDs again (may have changed)
        carla_pids=$(pgrep -f "CarlaUE4" 2>/dev/null || true)
        
        for pid in $carla_pids; do
            # Kill all children first
            local children=$(pgrep -P $pid 2>/dev/null || true)
            if [ -n "$children" ]; then
                for child in $children; do
                    kill -9 $child 2>/dev/null || true
                done
            fi
            # Force kill parent
            kill -9 $pid 2>/dev/null || true
        done
        
        # Also try pkill as fallback
        pkill -9 -f "CarlaUE4" 2>/dev/null || true
        
        sleep 3
        
        # Sync filesystem to prevent freeze
        sync 2>/dev/null || true
    fi
    
    # Final verification
    if pgrep -f "CarlaUE4" > /dev/null; then
        log_error "Failed to cleanup all CARLA processes"
        log_warning "Some processes may be in uninterruptible sleep (D state)"
        log_warning "You may need to wait or reboot if system is frozen"
        return 1
    else
        log_success "All CARLA processes cleaned up"
        # Sync filesystem to prevent freeze
        sync 2>/dev/null || true
        return 0
    fi
}

# Function: Start CARLA server (improved from auto_train.sh)
start_carla() {
    log "🚀 Starting CARLA server..."
    
    if [ "$(check_carla)" = "OK" ]; then
        log_success "CARLA already running"
        return 0
    fi
    
    # Cleanup old CARLA processes first (like auto_train.sh)
    cleanup_carla_processes
    
    sleep 2
    
    # Start CARLA (using Vulkan instead of OpenGL)
    cd "$CARLA_DIR"
    DISPLAY=:0 ./CarlaUE4.sh -quality-level=Low -no-sound -vulkan > /tmp/carla_server.log 2>&1 &
    CARLA_PID=$!
    
    log "CARLA started (PID: $CARLA_PID)"
    
    # Wait for CARLA to be ready
    log "⏳ Waiting for CARLA to be ready..."
    for i in {1..30}; do
        sleep 2
        if [ "$(check_carla)" = "OK" ]; then
            log_success "CARLA is ready!"
            return 0
        fi
        log "   Attempt $i/30..."
    done
    
    log_error "CARLA failed to start"
    return 1
}

# Step 1: IL Collection
step1_il_collection() {
    log_step "Step 1: IL Collection"
    
    # Check if demo file exists
    if ls "$RL_AGENT_DIR/expert_demonstrations"/*.npz 1> /dev/null 2>&1; then
        demo_file=$(ls "$RL_AGENT_DIR/expert_demonstrations"/*.npz | head -1)
        size=$(du -h "$demo_file" 2>/dev/null | cut -f1)
        log_success "Demo file already exists: $demo_file ($size)"
        log "Skipping collection..."
        return 0
    fi
    
    # Ensure CARLA is running and ready
    if [ "$(check_carla)" != "OK" ]; then
        start_carla || return 1
    fi
    
    # Wait a bit for CARLA to be fully ready
    log "⏳ Waiting for CARLA to be fully ready..."
    sleep 5
    
    log "Starting IL collection ($IL_NUM_EPISODES episodes)..."
    cd "$RL_AGENT_DIR"
    source venv/bin/activate
    
    # Increase timeout and add retry in script
    if timeout 600 python utils/collect_expert_demos.py \
        --num-episodes "$IL_NUM_EPISODES" \
        --town "$IL_TOWN" \
        --agent "$IL_AGENT" \
        > "$LOG_DIR/il_collection.log" 2>&1; then
        
        # Verify demo file
        if ls "$RL_AGENT_DIR/expert_demonstrations"/*.npz 1> /dev/null 2>&1; then
            demo_file=$(ls "$RL_AGENT_DIR/expert_demonstrations"/*.npz | head -1)
            size=$(du -h "$demo_file" 2>/dev/null | cut -f1)
            log_success "Collection complete: $demo_file ($size)"
            return 0
        else
            log_error "Collection completed but no demo file found"
            return 1
        fi
    else
        log_error "Collection failed"
        log "Check log: tail -20 $LOG_DIR/il_collection.log"
        return 1
    fi
}

# Step 2: IL Training
step2_il_training() {
    log_step "Step 2: IL Training"
    
    # Check if IL model exists
    if [ -f "$RL_AGENT_DIR/checkpoints/il_pretrained/best_il_model.pt" ]; then
        size=$(du -h "$RL_AGENT_DIR/checkpoints/il_pretrained/best_il_model.pt" 2>/dev/null | cut -f1)
        log_success "IL model already exists: best_il_model.pt ($size)"
        log "Skipping training..."
        return 0
    fi
    
    # Check demo file
    if ! ls "$RL_AGENT_DIR/expert_demonstrations"/*.npz 1> /dev/null 2>&1; then
        log_error "No demo file found. Run Step 1 first."
        return 1
    fi
    
    # Find latest demo file (by modification time, not by name)
    demo_file=$(ls -t "$RL_AGENT_DIR/expert_demonstrations"/*.npz 2>/dev/null | head -1)
    # Also check if we have the target number of episodes file
    target_file="$RL_AGENT_DIR/expert_demonstrations/demonstrations_${IL_NUM_EPISODES}episodes.npz"
    if [ -f "$target_file" ]; then
        demo_file="$target_file"
        log "Using target demo file: $target_file"
    else
        log "Using latest demo file: $demo_file"
    fi
    if [ -z "$demo_file" ] || [ ! -f "$demo_file" ]; then
        log_error "No demo file found. Run Step 1 first."
        return 1
    fi
    
    log "Starting IL training..."
    log "  Demo file: $demo_file"
    log "  Config: $CONFIG_FILE"
    
    cd "$RL_AGENT_DIR"
    source venv/bin/activate
    
    # Optimized settings for faster training
    # Reduced epochs: 50 -> 20 (still good quality, much faster)
    # Batch size: 8 (reduced from 16 to avoid GPU memory conflicts with RL training)
    # Use GPU for faster training (will auto-fallback to CPU if GPU busy)
    if python training/train_imitation_learning.py \
        --config "$CONFIG_FILE" \
        --demonstrations "$demo_file" \
        --output-dir "$RL_AGENT_DIR/checkpoints/il_pretrained" \
        --batch-size 8 \
        --num-epochs 20 \
        --device cpu \
        > "$LOG_DIR/il_training.log" 2>&1; then
        # Verify model
        if [ -f "$RL_AGENT_DIR/checkpoints/il_pretrained/best_il_model.pt" ]; then
            size=$(du -h "$RL_AGENT_DIR/checkpoints/il_pretrained/best_il_model.pt" 2>/dev/null | cut -f1)
            log_success "IL Training complete: best_il_model.pt ($size)"
            return 0
        else
            log_error "Training completed but no model file found"
            return 1
        fi
    else
        log_error "IL Training failed"
        log "Check log: tail -20 $LOG_DIR/il_training.log"
        return 1
    fi
}

# Step 3: Verify IL
step3_verify_il() {
    log_step "Step 3: Verify IL"
    
    # Check demo file
    if ls "$RL_AGENT_DIR/expert_demonstrations"/*.npz 1> /dev/null 2>&1; then
        demo_file=$(ls "$RL_AGENT_DIR/expert_demonstrations"/*.npz | head -1)
        size=$(du -h "$demo_file" 2>/dev/null | cut -f1)
        log_success "Demo file: $demo_file ($size)"
    else
        log_error "Demo file not found"
        return 1
    fi
    
    # Check IL model
    if [ -f "$RL_AGENT_DIR/checkpoints/il_pretrained/best_il_model.pt" ]; then
        size=$(du -h "$RL_AGENT_DIR/checkpoints/il_pretrained/best_il_model.pt" 2>/dev/null | cut -f1)
        log_success "IL Model: best_il_model.pt ($size)"
    else
        log_error "IL Model not found"
        return 1
    fi
    
    # Check config
    if grep -q "enabled: true" "$RL_AGENT_DIR/$CONFIG_FILE" 2>/dev/null; then
        log_success "IL enabled in config"
    else
        log_warning "IL not enabled in config (will use RL only)"
    fi
    
    log_success "IL verification complete"
    return 0
}

# Step 4: RL Training (with IL pre-training and CARLA auto-restart)
step4_rl_training() {
    log_step "Step 4: RL Training (with IL pre-training)"
    
    # Ensure CARLA is running
    if [ "$(check_carla)" != "OK" ]; then
        start_carla || return 1
    fi
    
    # Start CARLA monitoring in background (like auto_train.sh)
    (
        while true; do
            sleep 60  # Check every 60 seconds
            if [ "$(check_carla)" != "OK" ]; then
                log_error "CARLA stopped! Restarting..."
                cleanup_carla_processes
                start_carla || {
                    log_error "Failed to restart CARLA. Retrying in 30s..."
                    sleep 30
                    continue
                }
            fi
        done
    ) &
    CARLA_MONITOR_PID=$!
    log "CARLA monitor started (PID: $CARLA_MONITOR_PID) - will auto-restart if CARLA crashes"
    
    # Find latest checkpoint
    latest_checkpoint=""
    if [ -f "$RL_AGENT_DIR/checkpoints/enhanced"/*/model.zip ]; then
        latest_checkpoint=$(ls -t "$RL_AGENT_DIR/checkpoints/enhanced"/*/model.zip 2>/dev/null | head -1)
    elif [ -f "$RL_AGENT_DIR/checkpoints/best_model/best_model.zip" ]; then
        latest_checkpoint="$RL_AGENT_DIR/checkpoints/best_model/best_model.zip"
    fi
    
    log "Starting RL training..."
    log "  Total timesteps: $RL_TOTAL_TIMESTEPS"
    if [ -n "$latest_checkpoint" ]; then
        log "  Resuming from: $latest_checkpoint"
    else
        log "  Starting fresh (will use IL pre-training if available)"
    fi
    
    cd "$RL_AGENT_DIR"
    source venv/bin/activate
    
    # Build command
    cmd="python training/train.py --config $CONFIG_FILE --num-envs 1"
    if [ -n "$latest_checkpoint" ] && [ -f "$latest_checkpoint" ]; then
        cmd="$cmd --resume $latest_checkpoint"
    fi
    
    # Run training in background (like auto_train.sh) so pipeline can continue
    log "Starting RL training in background..."
    log "  Monitor PID: $CARLA_MONITOR_PID"
    log "  Training log: $LOG_DIR/rl_training.log"
    
    # Start training in background
    cd "$RL_AGENT_DIR"
    source venv/bin/activate
    $cmd > "$LOG_DIR/rl_training.log" 2>&1 &
    TRAINING_PID=$!
    
    log_success "RL Training started (PID: $TRAINING_PID)"
    log "  CARLA will auto-restart if it crashes"
    log "  Training will continue until completion or interruption"
    log ""
    log "To monitor training:"
    log "  tail -f $LOG_DIR/rl_training.log"
    log ""
    
    # Wait a bit and show initial training status
    sleep 5
    
    # Show initial training log (last 10 lines)
    if [ -f "$LOG_DIR/rl_training.log" ]; then
        log "📋 Initial Training Status:"
        tail -10 "$LOG_DIR/rl_training.log" 2>/dev/null | while IFS= read -r line; do
            # Filter and show important lines
            if echo "$line" | grep -qE "(INFO|WARNING|ERROR|IL pre-trained|Loading|Starting training|Step|Episode|reward)" 2>/dev/null; then
                log "   $line"
            fi
        done
        log ""
    fi
    
    # Start background log monitor (shows periodic updates) - Improved stability
    (
        last_line_count=0
        last_step=0
        last_checkpoint_time=0
        update_count=0
        
        while pgrep -f "train.py" > /dev/null; do
            sleep 30  # Update every 30 seconds (more stable)
            
            # Check if training process still exists
            if ! pgrep -f "train.py" > /dev/null; then
                break
            fi
            
            # Simple and robust log reading with error handling
            if [ -f "$LOG_DIR/rl_training.log" ]; then
                # Get current line count (with error handling)
                current_lines=$(wc -l < "$LOG_DIR/rl_training.log" 2>/dev/null || echo "0")
                
                # Only process if log has new content
                if [ "$current_lines" -gt "$last_line_count" ] && [ "$current_lines" -gt 0 ]; then
                    # Get current step (simple pattern, more reliable)
                    current_step=$(tail -30 "$LOG_DIR/rl_training.log" 2>/dev/null | grep "Callback: Step" | tail -1 | sed 's/.*Step \([0-9]*\).*/\1/' 2>/dev/null || echo "")
                    
                    # Only update if step changed significantly (avoid spam)
                    if [ -n "$current_step" ] && [ "$current_step" != "$last_step" ] && [ "$current_step" -gt "$last_step" ]; then
                        # Get episode info (simpler pattern)
                        episode_info=$(tail -100 "$LOG_DIR/rl_training.log" 2>/dev/null | grep -E "rollout/.*ep_rew_mean|Episode reward" | tail -1 || echo "")
                        
                        if [ -n "$episode_info" ]; then
                            # Extract reward if available
                            reward=$(echo "$episode_info" | grep -oE "ep_rew_mean[^|]*|Episode reward: -?[0-9.]+" | grep -oE "-?[0-9.]+" | head -1 || echo "")
                            if [ -n "$reward" ]; then
                                echo -e "${CYAN}[$(date '+%Y-%m-%d %H:%M:%S')]${NC}    [Training] Step $current_step: Reward $reward" | tee -a "$MASTER_LOG" 2>/dev/null || true
                            else
                                echo -e "${CYAN}[$(date '+%Y-%m-%d %H:%M:%S')]${NC}    [Training] Step $current_step: Training active" | tee -a "$MASTER_LOG" 2>/dev/null || true
                            fi
                        else
                            echo -e "${CYAN}[$(date '+%Y-%m-%d %H:%M:%S')]${NC}    [Training] Step $current_step: Training active" | tee -a "$MASTER_LOG" 2>/dev/null || true
                        fi
                        
                        last_step="$current_step"
                        update_count=$((update_count + 1))
                    fi
                    
                    # Check for checkpoint saves (simpler pattern)
                    checkpoint_time=$(stat -c %Y "$LOG_DIR/rl_training.log" 2>/dev/null || echo "0")
                    if [ "$checkpoint_time" -gt "$last_checkpoint_time" ]; then
                        # Check if checkpoint was saved recently
                        if tail -50 "$LOG_DIR/rl_training.log" 2>/dev/null | grep -qE "Saved checkpoint|checkpoint.*steps" 2>/dev/null; then
                            latest_cp=$(ls -t "$RL_AGENT_DIR/checkpoints/checkpoint"/*.zip 2>/dev/null | head -1)
                            if [ -n "$latest_cp" ]; then
                                cp_steps=$(basename "$latest_cp" | grep -oE "[0-9]+" | head -1 || echo "")
                                if [ -n "$cp_steps" ]; then
                                    echo -e "${CYAN}[$(date '+%Y-%m-%d %H:%M:%S')]${NC}    [Training] 💾 Checkpoint saved: $cp_steps steps" | tee -a "$MASTER_LOG" 2>/dev/null || true
                                fi
                            fi
                            last_checkpoint_time="$checkpoint_time"
                        fi
                    fi
                    
                    last_line_count=$current_lines
                fi
                
                # Heartbeat every 2 minutes (less frequent, more stable)
                if [ $((update_count % 4)) -eq 0 ] && [ "$update_count" -gt 0 ]; then
                    current_step=$(tail -30 "$LOG_DIR/rl_training.log" 2>/dev/null | grep "Callback: Step" | tail -1 | sed 's/.*Step \([0-9]*\).*/\1/' 2>/dev/null || echo "N/A")
                    echo -e "${CYAN}[$(date '+%Y-%m-%d %H:%M:%S')]${NC}    [Training] ⏳ Training in progress (Step: $current_step)" | tee -a "$MASTER_LOG" 2>/dev/null || true
                fi
            else
                # Log file doesn't exist yet, wait
                sleep 5
            fi
        done
        
        # Final message when training stops
        echo -e "${CYAN}[$(date '+%Y-%m-%d %H:%M:%S')]${NC}    [Training] ⚠️  Training process stopped" | tee -a "$MASTER_LOG" 2>/dev/null || true
    ) &
    LOG_MONITOR_PID=$!
    
    log "📊 Training log monitor started (PID: $LOG_MONITOR_PID) - will show updates every 30s"
    log ""
    
    # Return success immediately (training runs in background)
    # CARLA monitor and log monitor will continue running
    return 0
}

# Step 5: Start Dashboard
step5_start_dashboard() {
    log_step "Step 5: Start Dashboard"
    
    # Check if dashboard is already running
    if pgrep -f "web_dashboard/app.py" > /dev/null; then
        log_success "Dashboard already running"
        log "Dashboard URL: http://localhost:$DASHBOARD_PORT"
        return 0
    fi
    
    log "Starting dashboard..."
    cd "$RL_AGENT_DIR"
    source venv/bin/activate
    
    # Start dashboard in background
    nohup python web_dashboard/app.py > "$LOG_DIR/dashboard.log" 2>&1 &
    DASHBOARD_PID=$!
    
    # Wait for dashboard to start
    log "⏳ Waiting for dashboard to start..."
    for i in {1..10}; do
        sleep 1
        if curl -s http://localhost:$DASHBOARD_PORT/api/status > /dev/null 2>&1; then
            log_success "Dashboard started (PID: $DASHBOARD_PID)"
            log "Dashboard URL: http://localhost:$DASHBOARD_PORT"
            log "API: http://localhost:$DASHBOARD_PORT/api/status"
            return 0
        fi
    done
    
    log_warning "Dashboard may not be ready yet (check manually)"
    log "Dashboard URL: http://localhost:$DASHBOARD_PORT"
    return 0  # Don't fail - dashboard is optional
}

# Step 6: Evaluation/Test
step6_evaluation() {
    log_step "Step 6: Evaluation/Test"
    
    # Find best model
    model_path=""
    if [ -f "$RL_AGENT_DIR/checkpoints/best_model/best_model.zip" ]; then
        model_path="$RL_AGENT_DIR/checkpoints/best_model/best_model.zip"
    elif [ -f "$RL_AGENT_DIR/checkpoints/enhanced"/*/model.zip ]; then
        model_path=$(ls -t "$RL_AGENT_DIR/checkpoints/enhanced"/*/model.zip 2>/dev/null | head -1)
    elif [ -f "$RL_AGENT_DIR/checkpoints/final_model.zip" ]; then
        model_path="$RL_AGENT_DIR/checkpoints/final_model.zip"
    fi
    
    if [ -z "$model_path" ] || [ ! -f "$model_path" ]; then
        log_error "No trained model found for evaluation"
        log "Skipping evaluation..."
        return 1
    fi
    
    # Ensure CARLA is running
    if [ "$(check_carla)" != "OK" ]; then
        start_carla || return 1
    fi
    
    log "Running evaluation ($EVAL_NUM_EPISODES episodes)..."
    log "  Model: $model_path"
    
    cd "$RL_AGENT_DIR"
    source venv/bin/activate
    
    if python training/evaluate.py \
        --model "$model_path" \
        --config "$CONFIG_FILE" \
        --episodes "$EVAL_NUM_EPISODES" \
        > "$LOG_DIR/evaluation.log" 2>&1; then
        log_success "Evaluation complete"
        log "Results saved to: $LOG_DIR/evaluation.log"
        return 0
    else
        log_error "Evaluation failed"
        log "Check log: tail -20 $LOG_DIR/evaluation.log"
        return 1
    fi
}

# Function: Monitor and restart if needed
monitor_step() {
    local step_name="$1"
    local step_func="$2"
    local max_retries=3
    local retry_count=0
    
    while [ $retry_count -lt $max_retries ]; do
        if $step_func; then
            return 0
        else
            retry_count=$((retry_count + 1))
            if [ $retry_count -lt $max_retries ]; then
                log_warning "$step_name failed. Retrying ($retry_count/$max_retries)..."
                sleep 5
            else
                log_error "$step_name failed after $max_retries attempts"
                return 1
            fi
        fi
    done
}

# Handle stop command
handle_stop() {
    log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    log "🛑 Stopping Master Pipeline"
    log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    log ""
    safe_cleanup
    rm -f "$PID_FILE" "$STATE_FILE" 2>/dev/null
    log_success "Pipeline stopped"
    exit 0
}

# Main pipeline
main() {
    # Handle stop command
    if [ "$1" = "stop" ]; then
        handle_stop
    fi
    
    # Save PID
    echo $$ > "$PID_FILE"
    
    log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    log "🚀 Master Pipeline: IL → RL → Dashboard → Test"
    log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    log ""
    log "PID: $$"
    log "To stop: ./scripts/master_pipeline.sh stop"
    log ""
    
    # Load state (resume from last step if interrupted)
    current_step=$(load_state)
    log "Current state: $current_step"
    log ""
    
    # Step 1: IL Collection
    if [ "$current_step" = "step1" ] || [ "$current_step" = "start" ]; then
        if monitor_step "Step 1: IL Collection" step1_il_collection; then
            save_state "step2"
        else
            log_error "Pipeline stopped at Step 1"
            exit 1
        fi
    fi
    
    # Step 2: IL Training
    if [ "$current_step" = "step2" ] || [ "$current_step" = "step1" ]; then
        if monitor_step "Step 2: IL Training" step2_il_training; then
            save_state "step3"
        else
            log_error "Pipeline stopped at Step 2"
            exit 1
        fi
    fi
    
    # Step 3: Verify IL
    if [ "$current_step" = "step3" ] || [ "$current_step" = "step2" ]; then
        if monitor_step "Step 3: Verify IL" step3_verify_il; then
            save_state "step4"
        else
            log_error "Pipeline stopped at Step 3"
            exit 1
        fi
    fi
    
    # Update current_step after step3 completes
    current_step=$(load_state)
    
    # Step 4: RL Training (run after step3 completes)
    if [ "$current_step" = "step4" ] || [ "$current_step" = "step3" ]; then
        if monitor_step "Step 4: RL Training" step4_rl_training; then
            save_state "step5"
        else
            log_warning "RL Training interrupted (checkpoint may be saved)"
            save_state "step5"  # Continue to evaluation anyway
        fi
    fi
    
    # Step 5: Start Dashboard
    if [ "$current_step" = "step5" ] || [ "$current_step" = "step4" ]; then
        if monitor_step "Step 5: Start Dashboard" step5_start_dashboard; then
            save_state "step6"
        else
            log_warning "Dashboard start failed (but continuing)"
            save_state "step6"  # Continue anyway
        fi
    fi
    
    # Step 6: Evaluation
    if [ "$current_step" = "step6" ] || [ "$current_step" = "step5" ]; then
        if monitor_step "Step 6: Evaluation" step6_evaluation; then
            save_state "complete"
        else
            log_warning "Evaluation failed (but pipeline completed)"
        fi
    fi
    
    # Complete
    log ""
    log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    log_success "🎉 Master Pipeline Complete!"
    log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    log ""
    log "Summary:"
    log "  ✅ Step 1: IL Collection"
    log "  ✅ Step 2: IL Training"
    log "  ✅ Step 3: Verify IL"
    log "  ✅ Step 4: RL Training"
    log "  ✅ Step 5: Dashboard"
    log "  ✅ Step 6: Evaluation"
    log ""
    log "Dashboard:"
    log "  🌐 URL: http://localhost:$DASHBOARD_PORT"
    log "  📡 API: http://localhost:$DASHBOARD_PORT/api/status"
    log ""
    log "Logs:"
    log "  • Master log: $MASTER_LOG"
    log "  • IL Collection: $LOG_DIR/il_collection.log"
    log "  • IL Training: $LOG_DIR/il_training.log"
    log "  • RL Training: $LOG_DIR/rl_training.log"
    log "  • Dashboard: $LOG_DIR/dashboard.log"
    log "  • Evaluation: $LOG_DIR/evaluation.log"
    log ""
    
    # Cleanup state file
    rm -f "$STATE_FILE" "$PID_FILE"
    
    # Show cleanup options
    cleanup_on_complete
}

# Function: Safe cleanup all processes
safe_cleanup() {
    log ""
    log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    log "🛑 Safe Cleanup - Stopping all processes gracefully"
    log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    log ""
    
    # 1. Stop training processes (graceful)
    log "1. Stopping training processes..."
    if pgrep -f "train.py" > /dev/null; then
        pkill -TERM -f "train.py" 2>/dev/null || true
        log "   Sent TERM signal to training processes"
        
        # Wait for graceful shutdown
        local wait_count=0
        while [ $wait_count -lt 10 ] && pgrep -f "train.py" > /dev/null; do
            sleep 1
            wait_count=$((wait_count + 1))
        done
        
        # Force kill if still running
        if pgrep -f "train.py" > /dev/null; then
            log_warning "Training processes didn't stop, force killing..."
            pkill -9 -f "train.py" 2>/dev/null || true
        fi
        log_success "Training processes stopped"
    else
        log "   No training processes running"
    fi
    
    # 2. Stop IL processes (graceful)
    log ""
    log "2. Stopping IL processes..."
    if pgrep -f "collect_expert_demos\|train_imitation_learning" > /dev/null; then
        pkill -TERM -f "collect_expert_demos\|train_imitation_learning" 2>/dev/null || true
        sleep 2
        if pgrep -f "collect_expert_demos\|train_imitation_learning" > /dev/null; then
            pkill -9 -f "collect_expert_demos\|train_imitation_learning" 2>/dev/null || true
        fi
        log_success "IL processes stopped"
    else
        log "   No IL processes running"
    fi
    
    # 3. Stop dashboard (graceful)
    log ""
    log "3. Stopping dashboard..."
    if pgrep -f "web_dashboard/app.py" > /dev/null; then
        pkill -TERM -f "web_dashboard/app.py" 2>/dev/null || true
        sleep 1
        if pgrep -f "web_dashboard/app.py" > /dev/null; then
            pkill -9 -f "web_dashboard/app.py" 2>/dev/null || true
        fi
        log_success "Dashboard stopped"
    else
        log "   No dashboard running"
    fi
    
    # 4. Stop CARLA (graceful - but may need force)
    log ""
    log "4. Stopping CARLA..."
    if pgrep -f "CarlaUE4" > /dev/null; then
        # First, try to close any Python CARLA connections to prevent hanging
        python3 -c "
import carla
try:
    client = carla.Client('localhost', $CARLA_PORT)
    client.set_timeout(0.5)
    # Just accessing world and closing connection naturally
    world = client.get_world()
    del world
    del client
except:
    pass
" 2>/dev/null || true
        
        # Get all CARLA PIDs
        local carla_pids=$(pgrep -f "CarlaUE4" 2>/dev/null || true)
        
        # Try graceful first (kill children first, then parent)
        for pid in $carla_pids; do
            local children=$(pgrep -P $pid 2>/dev/null || true)
            if [ -n "$children" ]; then
                for child in $children; do
                    kill -TERM $child 2>/dev/null || true
                done
            fi
            kill -TERM $pid 2>/dev/null || true
        done
        
        sleep 3
        
        # Force kill if still running (kill all processes in tree)
        if pgrep -f "CarlaUE4" > /dev/null; then
            log_warning "CARLA didn't stop gracefully, force killing..."
            
            carla_pids=$(pgrep -f "CarlaUE4" 2>/dev/null || true)
            for pid in $carla_pids; do
                local children=$(pgrep -P $pid 2>/dev/null || true)
                if [ -n "$children" ]; then
                    for child in $children; do
                        kill -9 $child 2>/dev/null || true
                    done
                fi
                kill -9 $pid 2>/dev/null || true
            done
            
            # Fallback to pkill
            pkill -9 -f "CarlaUE4" 2>/dev/null || true
            sleep 2
        fi
        
        # Sync filesystem to prevent freeze
        sync 2>/dev/null || true
        
        log_success "CARLA stopped"
    else
        log "   No CARLA running"
    fi
    
    # 5. Cleanup PID files
    log ""
    log "5. Cleaning up PID files..."
    rm -f "$PID_FILE" 2>/dev/null
    log_success "PID files cleaned"
    
    # 6. Clear GPU memory (if available)
    log ""
    log "6. Clearing GPU memory..."
    if command -v nvidia-smi &> /dev/null; then
        # Just clear cache, don't reset GPU (requires root)
        python3 -c "import torch; torch.cuda.empty_cache() if torch.cuda.is_available() else None" 2>/dev/null || true
        log_success "GPU cache cleared"
    else
        log "   nvidia-smi not available"
    fi
    
    log ""
    log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    log_success "✅ Safe cleanup complete"
    log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    log ""
}

# Function: Cleanup on exit (save state)
cleanup_on_exit() {
    log_warning "Pipeline interrupted. State saved to: $STATE_FILE"
    log "Resume by running this script again (it will continue from last step)"
    # Don't cleanup processes on interrupt - let user decide
}

# Function: Cleanup on completion
cleanup_on_complete() {
    log ""
    log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    log "🎉 Pipeline Complete - Cleanup Options"
    log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    log ""
    log "Processes still running:"
    log "  • Dashboard: http://localhost:$DASHBOARD_PORT (useful for monitoring)"
    log "  • CARLA: May still be running (useful for evaluation)"
    log ""
    log "To cleanup all processes:"
    log "  ./scripts/master_pipeline.sh stop"
    log ""
    log "Or run safe cleanup manually:"
    log "  ./scripts/force_cleanup.sh"
    log ""
}

# Handle different exit scenarios
trap cleanup_on_exit EXIT INT TERM

# Run
main "$@"

