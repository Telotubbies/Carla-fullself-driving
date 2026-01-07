#!/bin/bash
# Script to stop auto_manage.sh gracefully

set -u

readonly RL_AGENT_DIR="/home/a/Desktop/CARLA_0.9.16/RL_Agent"
readonly PID_FILE="${RL_AGENT_DIR}/.auto_manage.pid"
readonly LOG_DIR="${RL_AGENT_DIR}/logs"
readonly AUTO_LOG="${LOG_DIR}/auto_manage.log"

log() {
    local message="$*"
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[${timestamp}] $message"
    echo "${timestamp} | INFO | STOP_SCRIPT | $message" >> "${AUTO_LOG}" 2>&1 || true
}

log_info() {
    log "ℹ️  $*"
}

log_success() {
    log "✅ $*"
}

log_warning() {
    log "⚠️  $*"
}

log_error() {
    log "❌ $*" >&2
}

# Check if auto_manage is running
check_auto_manage_running() {
    if [ ! -f "${PID_FILE}" ]; then
        return 1
    fi
    
    local pid=$(cat "${PID_FILE}" 2>/dev/null || echo "")
    if [ -z "${pid}" ]; then
        return 1
    fi
    
    if ps -p "${pid}" > /dev/null 2>&1; then
        return 0
    else
        # PID file exists but process is dead - clean up
        rm -f "${PID_FILE}"
        return 1
    fi
}

# Stop auto_manage gracefully
stop_auto_manage() {
    log_info "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    log_info "🛑 Stopping Auto Management Script"
    log_info "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    
    if ! check_auto_manage_running; then
        log_warning "Auto management script is not running"
        log_info "Checking for any remaining auto_manage processes..."
        
        # Check for any auto_manage processes
        local pids=$(pgrep -f "auto_manage.sh" 2>/dev/null || echo "")
        if [ -n "${pids}" ]; then
            log_warning "Found orphaned auto_manage processes: ${pids}"
            log_info "Killing orphaned processes..."
            for pid in ${pids}; do
                if ps -p "${pid}" > /dev/null 2>&1; then
                    log_info "   Killing PID ${pid}..."
                    kill -TERM "${pid}" 2>/dev/null || true
                    sleep 2
                    if ps -p "${pid}" > /dev/null 2>&1; then
                        log_warning "   Process ${pid} still alive, force killing..."
                        kill -9 "${pid}" 2>/dev/null || true
                    fi
                fi
            done
            log_success "Orphaned processes killed"
        else
            log_success "No auto_manage processes found"
        fi
        
        # Clean up PID file
        rm -f "${PID_FILE}"
        return 0
    fi
    
    local pid=$(cat "${PID_FILE}")
    log_info "Found auto_manage process (PID: ${pid})"
    
    # Try graceful shutdown first (SIGTERM)
    log_info "Sending SIGTERM to PID ${pid}..."
    if kill -TERM "${pid}" 2>/dev/null; then
        log_info "Waiting for graceful shutdown (max 10 seconds)..."
        
        # Wait for process to exit gracefully
        local wait_count=0
        while [ ${wait_count} -lt 10 ]; do
            if ! ps -p "${pid}" > /dev/null 2>&1; then
                log_success "Auto management script stopped gracefully"
                rm -f "${PID_FILE}"
                return 0
            fi
            sleep 1
            wait_count=$((wait_count + 1))
        done
        
        # If still running, force kill
        if ps -p "${pid}" > /dev/null 2>&1; then
            log_warning "Process did not exit gracefully, force killing..."
            kill -9 "${pid}" 2>/dev/null || true
            sleep 1
            
            if ! ps -p "${pid}" > /dev/null 2>&1; then
                log_success "Auto management script force killed"
                rm -f "${PID_FILE}"
                return 0
            else
                log_error "Failed to kill process ${pid}"
                return 1
            fi
        fi
    else
        log_error "Failed to send SIGTERM to PID ${pid}"
        return 1
    fi
}

# Main
main() {
    stop_auto_manage
    
    log_info "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    log_info "✅ Stop script completed"
    log_info "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
}

main "$@"

