#!/bin/bash
# Training Management Script - ตรวจสอบและจัดการ Training System
# ใช้สำหรับ: check, start, stop, restart, status, monitor

set -e

# Configuration
RL_AGENT_DIR="/home/a/Desktop/CARLA_0.9.16/RL_Agent"
CONFIG_FILE="config/phase1_accelerated_learning.yaml"
CARLA_DIR="/home/a/Desktop/CARLA_0.9.16"
CARLA_PORT=2000
LOG_DIR="$RL_AGENT_DIR/logs"
CHECKPOINT_DIR="$RL_AGENT_DIR/checkpoints_new/checkpoint"
TRAINING_LOG="$LOG_DIR/rl_training_new.log"
PID_FILE="$RL_AGENT_DIR/.training_new.pid"
STATUS_LOG="$LOG_DIR/process_status.log"
STATUS_LOGGER="$RL_AGENT_DIR/scripts/process_status_logger.sh"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# Functions
log() {
    echo -e "${CYAN}[$(date '+%Y-%m-%d %H:%M:%S')]${NC} $1"
}

log_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

log_error() {
    echo -e "${RED}❌ $1${NC}"
}

log_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

log_section() {
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

# Check CARLA
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
" 2>/dev/null || echo "NOT_RUNNING"
}

# Check training process
is_training_running() {
    if ps aux | grep -E "python.*train\.py.*$CONFIG_FILE" | grep -v grep | grep -qE "^\S+\s+\S+\s+\S+\s+\S+\s+\S+\s+\S+\s+\S+\s+\S+\s+\S+\s+\S+\s+python"; then
        return 0
    fi
    return 1
}

# Get training PID
get_training_pid() {
    # Find the actual python process running train.py
    ps aux | grep -E "python.*train\.py.*$CONFIG_FILE" | grep -v grep | awk '{print $2}' | head -1
}

# Log status to persistent log
log_to_status_file() {
    if [ -f "$STATUS_LOGGER" ]; then
        "$STATUS_LOGGER" "$@" 2>/dev/null || true
    fi
}

# Start CARLA
start_carla() {
    log_section "🚀 Starting CARLA Server"
    log_to_status_file carla
    
    if [ "$(check_carla)" = "OK" ]; then
        log_success "CARLA is already running"
        return 0
    fi
    
    log "Starting CARLA..."
    log_to_status_file carla
    cd "$CARLA_DIR" || { log_error "Failed to change to CARLA directory"; return 1; }
    pkill -f "CarlaUE4" 2>/dev/null || true
    sleep 2
    
    DISPLAY=:0 ./CarlaUE4.sh -quality-level=Low -no-sound -vulkan > /tmp/carla_server.log 2>&1 &
    CARLA_PID=$!
    
    log "CARLA started (PID: $CARLA_PID), waiting for ready..."
    
    for i in {1..30}; do
        sleep 2
        if [ "$(check_carla)" = "OK" ]; then
            log_success "CARLA is ready!"
            log_to_status_file carla
            return 0
        fi
        log "   Attempt $i/30..."
    done
    
    log_error "CARLA failed to start"
    log_to_status_file carla
    return 1
}

# Stop CARLA
stop_carla() {
    log_section "🛑 Stopping CARLA Server"
    
    if [ "$(check_carla)" != "OK" ]; then
        log_warning "CARLA is not running"
        return 0
    fi
    
    log "Stopping CARLA..."
    pkill -f "CarlaUE4" 2>/dev/null || true
    sleep 3
    
    if pgrep -f "CarlaUE4" > /dev/null; then
        log_warning "Force killing CARLA..."
        pkill -9 -f "CarlaUE4" 2>/dev/null || true
        sleep 2
    fi
    
    if [ "$(check_carla)" != "OK" ]; then
        log_success "CARLA stopped"
        return 0
    else
        log_error "Failed to stop CARLA"
        return 1
    fi
}

# Start training
start_training() {
    log_section "🚀 Starting Training"
    log_to_status_file training
    
    if is_training_running; then
        PID=$(get_training_pid)
        log_warning "Training is already running (PID: $PID)"
        log_to_status_file training
        return 0
    fi
    
    # Check CARLA
    if [ "$(check_carla)" != "OK" ]; then
        log "CARLA not running, starting..."
        start_carla || { log_to_status_file training; return 1; }
    fi
    
    # Find latest checkpoint
    latest_checkpoint=""
    if [ -d "$CHECKPOINT_DIR" ]; then
        latest_checkpoint=$(ls -t "$CHECKPOINT_DIR"/*.zip 2>/dev/null | head -1)
    fi
    
    cd "$RL_AGENT_DIR" || { log_error "Failed to change to RL_AGENT directory"; log_to_status_file training; return 1; }
    
    if [ ! -f "venv/bin/activate" ]; then
        log_error "Virtual environment not found"
        log_to_status_file training
        return 1
    fi
    source venv/bin/activate
    
    cmd="python training/train.py --config $CONFIG_FILE --num-envs 1"
    if [ -n "$latest_checkpoint" ] && [ -f "$latest_checkpoint" ]; then
        checkpoint_name=$(basename "$latest_checkpoint")
        checkpoint_steps=$(echo "$checkpoint_name" | grep -oP "rl_model_\K\d+" || echo "0")
        log "📂 Resuming from checkpoint: $checkpoint_name (${checkpoint_steps} steps)"
        cmd="$cmd --resume $latest_checkpoint"
    else
        log "🆕 Starting fresh (from 0)"
    fi
    
    # Start training
    nohup $cmd > "$TRAINING_LOG" 2>&1 &
    TRAIN_PID=$!
    
    sleep 5
    PYTHON_PID=$(get_training_pid)
    
    if [ -n "$PYTHON_PID" ]; then
        echo "$PYTHON_PID" > "$PID_FILE"
        log_success "Training started (PID: $PYTHON_PID)"
        log "📝 Log: $TRAINING_LOG"
        log_to_status_file training
        return 0
    else
        log_error "Failed to start training"
        log_to_status_file training
        return 1
    fi
}

# Stop training
stop_training() {
    log_section "🛑 Stopping Training"
    
    if ! is_training_running; then
        log_warning "Training is not running"
        return 0
    fi
    
    PID=$(get_training_pid)
    log "Stopping training (PID: $PID)..."
    
    kill "$PID" 2>/dev/null || true
    sleep 3
    
    if ps -p "$PID" > /dev/null 2>&1; then
        log_warning "Force killing..."
        kill -9 "$PID" 2>/dev/null || true
        sleep 1
    fi
    
    pkill -f "train.py.*$CONFIG_FILE" 2>/dev/null || true
    rm -f "$PID_FILE"
    
    log_success "Training stopped"
    return 0
}

# Restart training
restart_training() {
    log_section "🔄 Restarting Training"
    stop_training
    sleep 2
    start_training
}

# Restart CARLA
restart_carla() {
    log_section "🔄 Restarting CARLA"
    stop_carla
    sleep 2
    start_carla
}

# Restart all
restart_all() {
    log_section "🔄 Restarting All (CARLA + Training)"
    stop_training
    stop_carla
    sleep 3
    start_carla
    sleep 5
    start_training
}

# Show status
show_status() {
    log_section "📊 System Status"
    
    # Log current status to persistent log
    log_to_status_file collect
    
    echo ""
    
    # CARLA Status
    echo "📡 CARLA Server:"
    if [ "$(check_carla)" = "OK" ]; then
        CARLA_PID=$(pgrep -f "CarlaUE4" | head -1 || echo "")
        if [ -n "$CARLA_PID" ]; then
            CPU=$(ps -p "$CARLA_PID" -o %cpu= 2>/dev/null | tr -d ' ' || echo "N/A")
            MEM=$(ps -p "$CARLA_PID" -o %mem= 2>/dev/null | tr -d ' ' || echo "N/A")
            TIME=$(ps -p "$CARLA_PID" -o etime= 2>/dev/null | tr -d ' ' || echo "N/A")
            log_success "Running (PID: $CARLA_PID, CPU: ${CPU}%, MEM: ${MEM}%, Time: $TIME)"
        else
            log_success "Running"
        fi
    else
        log_error "Not running"
    fi
    echo ""
    
    # Training Status
    echo "🤖 Training Client:"
    if is_training_running; then
        PID=$(get_training_pid)
        if [ -n "$PID" ]; then
            CPU=$(ps -p "$PID" -o %cpu= 2>/dev/null | tr -d ' ' || echo "N/A")
            MEM=$(ps -p "$PID" -o %mem= 2>/dev/null | tr -d ' ' || echo "N/A")
            TIME=$(ps -p "$PID" -o etime= 2>/dev/null | tr -d ' ' || echo "N/A")
            
            # Get latest step from log
            latest_step="N/A"
            if [ -f "$TRAINING_LOG" ]; then
                latest_step=$(tail -100 "$TRAINING_LOG" 2>/dev/null | grep -oP "Callback: Step \K\d+" | tail -1 || echo "N/A")
            fi
            
            log_success "Running (PID: $PID, CPU: ${CPU}%, MEM: ${MEM}%, Time: $TIME, Step: $latest_step)"
        else
            log_success "Running"
        fi
    else
        log_error "Not running"
    fi
    echo ""
    
    # Latest Checkpoint
    echo "📂 Latest Checkpoint:"
    if [ -d "$CHECKPOINT_DIR" ]; then
        latest_checkpoint=$(ls -t "$CHECKPOINT_DIR"/*.zip 2>/dev/null | head -1)
        if [ -n "$latest_checkpoint" ]; then
            checkpoint_name=$(basename "$latest_checkpoint")
            checkpoint_steps=$(echo "$checkpoint_name" | grep -oP "rl_model_\K\d+" || echo "0")
            checkpoint_time=$(stat -c "%y" "$latest_checkpoint" 2>/dev/null | cut -d'.' -f1 || echo "N/A")
            echo "   File: $checkpoint_name"
            echo "   Steps: $checkpoint_steps"
            echo "   Time: $checkpoint_time"
        else
            echo "   No checkpoints yet"
        fi
    else
        echo "   Checkpoint directory not found"
    fi
    echo ""
    
    # Log Status
    echo "📝 Log File:"
    if [ -f "$TRAINING_LOG" ]; then
        log_size=$(du -h "$TRAINING_LOG" | cut -f1)
        log_lines=$(wc -l < "$TRAINING_LOG" 2>/dev/null || echo "0")
        log_time=$(stat -c "%y" "$TRAINING_LOG" 2>/dev/null | cut -d'.' -f1 || echo "N/A")
        echo "   File: $(basename "$TRAINING_LOG")"
        echo "   Size: $log_size"
        echo "   Lines: $log_lines"
        echo "   Last update: $log_time"
    else
        echo "   Log file not found"
    fi
    echo ""
    
    # System Resources
    echo "📊 System Resources:"
    CPU_LOAD=$(cat /proc/loadavg | awk '{print $1, $2, $3}')
    MEM_INFO=$(free -h | awk '/Mem:/ {print "Used: " $3 " / " $2 " (" $3/$2*100 "%)"}')
    echo "   CPU Load: $CPU_LOAD"
    echo "   Memory: $MEM_INFO"
    
    if command -v nvidia-smi &> /dev/null; then
        GPU_INFO=$(nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu --format=csv,noheader,nounits | head -1 | awk -F', ' '{printf "Usage=%s%%, Mem=%s/%s MB, Temp=%s°C", $1, $2, $3, $4}')
        echo "   GPU: $GPU_INFO"
    elif command -v rocm-smi &> /dev/null; then
        GPU_USE=$(rocm-smi --showuse --showmeminfo vram --csv 2>/dev/null | grep -v "GPU" | head -1 | awk -F',' '{print $2}')
        GPU_MEM=$(rocm-smi --showuse --showmeminfo vram --csv 2>/dev/null | grep -v "GPU" | head -1 | awk -F',' '{print $4 "/" $3 " MB"}')
        GPU_TEMP=$(rocm-smi --showtemp --json 2>/dev/null | python3 -c "import sys, json; data=json.load(sys.stdin); print(next((v.get('Temperature (Sensor junction) (C)', v.get('Temperature (Sensor edge) (C)', 'N/A')) for k,v in data.items() if 'card' in k.lower()), 'N/A'))" 2>/dev/null || echo "N/A")
        if [ "${GPU_TEMP}" != "N/A" ] && [ "${GPU_TEMP}" != "None" ] && [ -n "${GPU_TEMP}" ]; then
            echo "   GPU: Use: ${GPU_USE}%, VRAM: ${GPU_MEM}, Temp: ${GPU_TEMP}°C"
        else
            echo "   GPU: Use: ${GPU_USE}%, VRAM: ${GPU_MEM}"
        fi
    fi
    echo ""
}

# Monitor training (check and auto-restart)
monitor_training() {
    log_section "📊 Training Monitor"
    
    readonly CHECK_INTERVAL=60  # Check every 60 seconds
    readonly STUCK_THRESHOLD=30  # Consider stuck if no log update for 30 minutes
    readonly STATUS_LOG_INTERVAL=300  # Log status every 5 minutes
    
    log "Starting monitor (check every ${CHECK_INTERVAL}s)..."
    log "Stuck threshold: ${STUCK_THRESHOLD} minutes"
    log "Status logging: every ${STATUS_LOG_INTERVAL}s"
    log ""
    
    local last_status_log=0
    
    while true; do
        sleep "$CHECK_INTERVAL"
        
        # Log status periodically
        local now=$(date +%s)
        if [ $((now - last_status_log)) -ge "$STATUS_LOG_INTERVAL" ]; then
            log_to_status_file collect
            last_status_log=$now
        fi
        
        # Check CARLA
        if [ "$(check_carla)" != "OK" ]; then
            log_warning "CARLA stopped! Restarting..."
            log_to_status_file carla
            start_carla || {
                log_error "Failed to restart CARLA"
                log_to_status_file carla
                sleep 30
                continue
            }
        fi
        
        # Check training
        if ! is_training_running; then
            log_warning "Training stopped! Restarting..."
            log_to_status_file training
            start_training || {
                log_error "Failed to restart training"
                log_to_status_file training
                sleep 30
                continue
            }
        else
            # Check if training is stuck
            if [ -f "$TRAINING_LOG" ]; then
                local mtime=$(stat -c "%Y" "$TRAINING_LOG" 2>/dev/null || echo "0")
                local now=$(date +%s)
                local diff_minutes=$(( (now - mtime) / 60 ))
                
                if [ "$diff_minutes" -gt "$STUCK_THRESHOLD" ]; then
                    local PID=$(get_training_pid)
                    log_warning "Training may be stuck (no log update for ${diff_minutes} minutes)"
                    log "   Process PID: $PID"
                    log "   Restarting training..."
                    log_to_status_file training
                    restart_training
                fi
            fi
        fi
        
        # Show status every 5 minutes
        if [ $(($(date +%s) % 300)) -eq 0 ]; then
            local PID=$(get_training_pid)
            if [ -n "$PID" ]; then
                local CPU=$(ps -p "$PID" -o %cpu= 2>/dev/null | tr -d ' ' || echo "N/A")
                local latest_step=$(tail -100 "$TRAINING_LOG" 2>/dev/null | grep -oP "Callback: Step \K\d+" | tail -1 || echo "N/A")
                log "📊 Status: PID=$PID, CPU=${CPU}%, Step=$latest_step"
            fi
        fi
    done
}

# Main
case "$1" in
    check|status)
        show_status
        ;;
    start)
        start_training
        ;;
    stop)
        stop_training
        ;;
    restart)
        restart_training
        ;;
    start-carla)
        start_carla
        ;;
    stop-carla)
        stop_carla
        ;;
    restart-carla)
        restart_carla
        ;;
    restart-all)
        restart_all
        ;;
    monitor)
        monitor_training
        ;;
    view-log)
        if [ -f "$STATUS_LOG" ]; then
            if command -v "$RL_AGENT_DIR/scripts/view_status_log.sh" > /dev/null 2>&1; then
                "$RL_AGENT_DIR/scripts/view_status_log.sh" "${@:2}"
            else
                tail -50 "$STATUS_LOG"
            fi
        else
            log_error "Status log not found: $STATUS_LOG"
            exit 1
        fi
        ;;
    *)
        echo "Usage: $0 {check|status|start|stop|restart|start-carla|stop-carla|restart-carla|restart-all|monitor|view-log}"
        echo ""
        echo "Commands:"
        echo "  check, status    - Show system status"
        echo "  start            - Start training"
        echo "  stop             - Stop training"
        echo "  restart          - Restart training"
        echo "  start-carla      - Start CARLA server"
        echo "  stop-carla       - Stop CARLA server"
        echo "  restart-carla    - Restart CARLA server"
        echo "  restart-all     - Restart CARLA + Training"
        echo "  monitor         - Monitor and auto-restart (runs forever)"
        echo "  view-log         - View process status log (use -h for options)"
        echo ""
        exit 1
        ;;
esac

exit 0

