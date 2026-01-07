#!/bin/bash
# Process Status Logger - Logs status of all processes to a persistent log file
# This script provides structured logging for process monitoring

set -u  # Fail on undefined variables (more strict than set -e for this use case)

# ============================================================================
# CONFIGURATION
# ============================================================================

readonly RL_AGENT_DIR="/home/a/Desktop/CARLA_0.9.16/RL_Agent"
readonly CONFIG_FILE="config/phase1_accelerated_learning.yaml"
readonly CARLA_DIR="/home/a/Desktop/CARLA_0.9.16"
readonly CARLA_PORT=2000
readonly LOG_DIR="${RL_AGENT_DIR}/logs"
readonly STATUS_LOG="${LOG_DIR}/process_status.log"
readonly CHECKPOINT_DIR="${RL_AGENT_DIR}/checkpoints_new/checkpoint"
readonly TRAINING_LOG="${LOG_DIR}/rl_training_new.log"

# Ensure log directory exists
mkdir -p "${LOG_DIR}"

# ============================================================================
# LOGGING FUNCTIONS
# ============================================================================

log_status() {
    local level="$1"
    shift
    local message="$*"
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    
    # Format: TIMESTAMP | LEVEL | MESSAGE
    echo "${timestamp} | ${level} | ${message}" >> "${STATUS_LOG}"
}

log_info() {
    log_status "INFO" "$@"
}

log_warning() {
    log_status "WARNING" "$@"
}

log_error() {
    log_status "ERROR" "$@"
}

log_success() {
    log_status "SUCCESS" "$@"
}

# ============================================================================
# PROCESS DETECTION FUNCTIONS
# ============================================================================

check_carla_status() {
    python3 -c "
import carla
import sys
try:
    client = carla.Client('localhost', ${CARLA_PORT})
    client.set_timeout(2.0)
    world = client.get_world()
    map_name = world.get_map().name
    actors_count = len(world.get_actors())
    print(f'OK|{map_name}|{actors_count}')
except Exception as e:
    print(f'NOT_RUNNING|{str(e)}')
" 2>/dev/null || echo "NOT_RUNNING|Connection failed"
}

get_carla_pids() {
    pgrep -f "CarlaUE4" 2>/dev/null || true
}

get_training_pid() {
    ps aux 2>/dev/null | grep -E "python.*train\.py.*${CONFIG_FILE}" | grep -v grep | awk '{print $2}' | head -1
}

is_training_running() {
    local pid=$(get_training_pid)
    if [ -n "${pid}" ] && ps -p "${pid}" > /dev/null 2>&1; then
        return 0
    fi
    return 1
}

get_process_info() {
    local pid="$1"
    if [ -z "${pid}" ] || ! ps -p "${pid}" > /dev/null 2>&1; then
        echo "N/A|N/A|N/A|N/A"
        return 1
    fi
    
    local cpu=$(ps -p "${pid}" -o %cpu= 2>/dev/null | tr -d ' ' || echo "N/A")
    local mem=$(ps -p "${pid}" -o %mem= 2>/dev/null | tr -d ' ' || echo "N/A")
    local time=$(ps -p "${pid}" -o etime= 2>/dev/null | tr -d ' ' || echo "N/A")
    local state=$(ps -p "${pid}" -o stat= 2>/dev/null | tr -d ' ' || echo "N/A")
    
    echo "${cpu}|${mem}|${time}|${state}"
}

get_latest_training_step() {
    if [ ! -f "${TRAINING_LOG}" ]; then
        echo "N/A"
        return 1
    fi
    
    tail -100 "${TRAINING_LOG}" 2>/dev/null | grep -oP "Callback: Step \K\d+" | tail -1 || echo "N/A"
}

get_latest_checkpoint() {
    if [ ! -d "${CHECKPOINT_DIR}" ]; then
        echo "N/A|N/A"
        return 1
    fi
    
    local latest=$(ls -t "${CHECKPOINT_DIR}"/*.zip 2>/dev/null | head -1)
    if [ -z "${latest}" ]; then
        echo "N/A|N/A"
        return 1
    fi
    
    local name=$(basename "${latest}")
    local steps=$(echo "${name}" | grep -oP "rl_model_\K\d+" || echo "0")
    local mtime=$(stat -c "%y" "${latest}" 2>/dev/null | cut -d'.' -f1 || echo "N/A")
    
    echo "${name}|${steps}|${mtime}"
}

get_system_resources() {
    local cpu_load=$(cat /proc/loadavg 2>/dev/null | awk '{print $1, $2, $3}' || echo "N/A")
    local mem_info=$(free -h 2>/dev/null | awk '/Mem:/ {print $3 "/" $2 " (" $3/$2*100 "%)"}' || echo "N/A")
    local gpu_info="N/A"
    local gpu_temp="N/A"
    
    if command -v nvidia-smi &> /dev/null; then
        gpu_info=$(nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total --format=csv,noheader,nounits 2>/dev/null | head -1 || echo "N/A")
        gpu_temp=$(nvidia-smi --query-gpu=temperature.gpu --format=csv,noheader,nounits 2>/dev/null | head -1 | tr -d ' ' || echo "N/A")
        if [ "${gpu_temp}" != "N/A" ] && [ -n "${gpu_temp}" ]; then
            gpu_info="${gpu_info}, Temp=${gpu_temp}°C"
        fi
    elif command -v rocm-smi &> /dev/null; then
        gpu_info=$(rocm-smi --showuse --showmeminfo vram --csv 2>/dev/null | grep -v "GPU" | head -1 | awk -F',' '{print $2 "%, " $4 "/" $3 " MB"}' || echo "N/A")
        # Get temperature from rocm-smi JSON
        gpu_temp=$(rocm-smi --showtemp --json 2>/dev/null | python3 -c "import sys, json; data=json.load(sys.stdin); print(next((v.get('Temperature (Sensor junction) (C)', v.get('Temperature (Sensor edge) (C)', 'N/A')) for k,v in data.items() if 'card' in k.lower()), 'N/A'))" 2>/dev/null || echo "N/A")
        if [ "${gpu_temp}" != "N/A" ] && [ -n "${gpu_temp}" ] && [ "${gpu_temp}" != "None" ]; then
            gpu_info="${gpu_info}, Temp=${gpu_temp}°C"
        fi
    fi
    
    echo "${cpu_load}|${mem_info}|${gpu_info}"
}

# ============================================================================
# STATUS COLLECTION
# ============================================================================

collect_carla_status() {
    local carla_status=$(check_carla_status)
    local status=$(echo "${carla_status}" | cut -d'|' -f1)
    local map_name=$(echo "${carla_status}" | cut -d'|' -f2)
    local actors=$(echo "${carla_status}" | cut -d'|' -f3)
    
    local pids=$(get_carla_pids)
    local pid_count=$(echo "${pids}" | wc -w)
    
    if [ "${status}" = "OK" ]; then
        local main_pid=$(echo "${pids}" | head -1)
        local process_info=$(get_process_info "${main_pid}")
        local cpu=$(echo "${process_info}" | cut -d'|' -f1)
        local mem=$(echo "${process_info}" | cut -d'|' -f2)
        local time=$(echo "${process_info}" | cut -d'|' -f3)
        
        log_info "CARLA|RUNNING|PID=${main_pid}|CPU=${cpu}%|MEM=${mem}%|TIME=${time}|MAP=${map_name}|ACTORS=${actors}"
    else
        log_warning "CARLA|NOT_RUNNING|REASON=${map_name}"
    fi
}

collect_training_status() {
    if is_training_running; then
        local pid=$(get_training_pid)
        local process_info=$(get_process_info "${pid}")
        local cpu=$(echo "${process_info}" | cut -d'|' -f1)
        local mem=$(echo "${process_info}" | cut -d'|' -f2)
        local time=$(echo "${process_info}" | cut -d'|' -f3)
        local state=$(echo "${process_info}" | cut -d'|' -f4)
        
        local step=$(get_latest_training_step)
        local checkpoint_info=$(get_latest_checkpoint)
        local checkpoint_name=$(echo "${checkpoint_info}" | cut -d'|' -f1)
        local checkpoint_steps=$(echo "${checkpoint_info}" | cut -d'|' -f2)
        
        log_info "TRAINING|RUNNING|PID=${pid}|CPU=${cpu}%|MEM=${mem}%|TIME=${time}|STATE=${state}|STEP=${step}|CHECKPOINT=${checkpoint_steps}"
    else
        log_warning "TRAINING|NOT_RUNNING"
    fi
}

collect_system_status() {
    local resources=$(get_system_resources)
    local cpu_load=$(echo "${resources}" | cut -d'|' -f1)
    local mem_info=$(echo "${resources}" | cut -d'|' -f2)
    local gpu_info=$(echo "${resources}" | cut -d'|' -f3)
    
    log_info "SYSTEM|CPU_LOAD=${cpu_load}|MEM=${mem_info}|GPU=${gpu_info}"
}

collect_all_status() {
    log_info "STATUS_CHECK|START"
    collect_carla_status
    collect_training_status
    collect_system_status
    log_info "STATUS_CHECK|END"
}

# ============================================================================
# MAIN
# ============================================================================

main() {
    local command="${1:-collect}"
    
    case "${command}" in
        collect)
            collect_all_status
            ;;
        carla)
            collect_carla_status
            ;;
        training)
            collect_training_status
            ;;
        system)
            collect_system_status
            ;;
        *)
            echo "Usage: $0 {collect|carla|training|system}"
            echo ""
            echo "Commands:"
            echo "  collect   - Collect all status (default)"
            echo "  carla     - Collect CARLA status only"
            echo "  training  - Collect training status only"
            echo "  system    - Collect system resources only"
            exit 1
            ;;
    esac
}

main "$@"

