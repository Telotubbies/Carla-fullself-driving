#!/bin/bash
# View Status Log - Display process status log in a readable format

set -u

readonly RL_AGENT_DIR="/home/a/Desktop/CARLA_0.9.16/RL_Agent"
readonly STATUS_LOG="${RL_AGENT_DIR}/logs/process_status.log"

# Colors
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly CYAN='\033[0;36m'
readonly NC='\033[0m'

show_help() {
    cat << EOF
Usage: $0 [OPTIONS]

Options:
  -n, --lines N       Show last N lines (default: 50)
  -f, --follow        Follow log file (like tail -f)
  -a, --all           Show all log entries
  -c, --carla         Show only CARLA entries
  -t, --training       Show only training entries
  -s, --system         Show only system entries
  -e, --errors         Show only errors and warnings
  -h, --help          Show this help message

Examples:
  $0                  # Show last 50 lines
  $0 -n 100           # Show last 100 lines
  $0 -f               # Follow log file
  $0 -c               # Show only CARLA status
  $0 -e               # Show only errors/warnings
EOF
}

format_log_line() {
    local line="$1"
    local timestamp=$(echo "${line}" | cut -d'|' -f1)
    local level=$(echo "${line}" | cut -d'|' -f2)
    local message=$(echo "${line}" | cut -d'|' -f3-)
    
    local color="${NC}"
    case "${level}" in
        ERROR)
            color="${RED}"
            ;;
        WARNING)
            color="${YELLOW}"
            ;;
        SUCCESS)
            color="${GREEN}"
            ;;
        INFO)
            color="${CYAN}"
            ;;
    esac
    
    # Format based on message type
    if echo "${message}" | grep -q "^CARLA|"; then
        format_carla_message "${message}"
    elif echo "${message}" | grep -q "^TRAINING|"; then
        format_training_message "${message}"
    elif echo "${message}" | grep -q "^SYSTEM|"; then
        format_system_message "${message}"
    else
        echo -e "${color}[${timestamp}] ${level}: ${message}${NC}"
    fi
}

format_carla_message() {
    local message="$1"
    local status=$(echo "${message}" | grep -oP "RUNNING|NOT_RUNNING")
    
    if [ "${status}" = "RUNNING" ]; then
        local pid=$(echo "${message}" | grep -oP "PID=\K[^|]+")
        local cpu=$(echo "${message}" | grep -oP "CPU=\K[^|]+")
        local mem=$(echo "${message}" | grep -oP "MEM=\K[^|]+")
        local time=$(echo "${message}" | grep -oP "TIME=\K[^|]+")
        local map=$(echo "${message}" | grep -oP "MAP=\K[^|]+")
        local actors=$(echo "${message}" | grep -oP "ACTORS=\K[^|]+")
        
        echo -e "${GREEN}[CARLA] ✅ Running${NC}"
        echo -e "   PID: ${pid} | CPU: ${cpu}% | MEM: ${mem}% | Time: ${time}"
        echo -e "   Map: ${map} | Actors: ${actors}"
    else
        local reason=$(echo "${message}" | grep -oP "REASON=\K.*" || echo "Unknown")
        echo -e "${RED}[CARLA] ❌ Not Running${NC}"
        echo -e "   Reason: ${reason}"
    fi
}

format_training_message() {
    local message="$1"
    local status=$(echo "${message}" | grep -oP "RUNNING|NOT_RUNNING")
    
    if [ "${status}" = "RUNNING" ]; then
        local pid=$(echo "${message}" | grep -oP "PID=\K[^|]+")
        local cpu=$(echo "${message}" | grep -oP "CPU=\K[^|]+")
        local mem=$(echo "${message}" | grep -oP "MEM=\K[^|]+")
        local time=$(echo "${message}" | grep -oP "TIME=\K[^|]+")
        local state=$(echo "${message}" | grep -oP "STATE=\K[^|]+")
        local step=$(echo "${message}" | grep -oP "STEP=\K[^|]+")
        local checkpoint=$(echo "${message}" | grep -oP "CHECKPOINT=\K[^|]+")
        
        echo -e "${GREEN}[TRAINING] ✅ Running${NC}"
        echo -e "   PID: ${pid} | CPU: ${cpu}% | MEM: ${mem}% | Time: ${time} | State: ${state}"
        echo -e "   Step: ${step} | Checkpoint: ${checkpoint}"
    else
        echo -e "${RED}[TRAINING] ❌ Not Running${NC}"
    fi
}

format_system_message() {
    local message="$1"
    local cpu=$(echo "${message}" | grep -oP "CPU_LOAD=\K[^|]+")
    local mem=$(echo "${message}" | grep -oP "MEM=\K[^|]+")
    local gpu=$(echo "${message}" | grep -oP "GPU=\K.*" || echo "N/A")
    
    echo -e "${CYAN}[SYSTEM]${NC}"
    echo -e "   CPU Load: ${cpu}"
    echo -e "   Memory: ${mem}"
    echo -e "   GPU: ${gpu}"
}

main() {
    local lines=50
    local follow=false
    local filter=""
    local show_all=false
    
    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -n|--lines)
                lines="$2"
                shift 2
                ;;
            -f|--follow)
                follow=true
                shift
                ;;
            -a|--all)
                show_all=true
                shift
                ;;
            -c|--carla)
                filter="CARLA"
                shift
                ;;
            -t|--training)
                filter="TRAINING"
                shift
                ;;
            -s|--system)
                filter="SYSTEM"
                shift
                ;;
            -e|--errors)
                filter="ERROR|WARNING"
                shift
                ;;
            -h|--help)
                show_help
                exit 0
                ;;
            *)
                echo "Unknown option: $1"
                show_help
                exit 1
                ;;
        esac
    done
    
    if [ ! -f "${STATUS_LOG}" ]; then
        echo "Status log not found: ${STATUS_LOG}"
        exit 1
    fi
    
    if [ "${follow}" = true ]; then
        tail -f "${STATUS_LOG}" | while IFS= read -r line; do
            if [ -n "${filter}" ] && ! echo "${line}" | grep -qE "${filter}"; then
                continue
            fi
            format_log_line "${line}"
            echo ""
        done
    elif [ "${show_all}" = true ]; then
        cat "${STATUS_LOG}" | while IFS= read -r line; do
            if [ -n "${filter}" ] && ! echo "${line}" | grep -oP "^[^|]+\|[^|]+\|" | grep -qE "${filter}"; then
                continue
            fi
            format_log_line "${line}"
            echo ""
        done
    else
        tail -n "${lines}" "${STATUS_LOG}" | while IFS= read -r line; do
            if [ -n "${filter}" ] && ! echo "${line}" | grep -oP "^[^|]+\|[^|]+\|" | grep -qE "${filter}"; then
                continue
            fi
            format_log_line "${line}"
            echo ""
        done
    fi
}

main "$@"

