#!/bin/bash
# System Watchdog Script - Prevents system freezes by monitoring and auto-rebooting
# This script monitors system responsiveness and reboots if the system is frozen

set -u

readonly SCRIPT_DIR="/home/a/Desktop/CARLA_0.9.16/RL_Agent/scripts"
readonly LOG_DIR="/home/a/Desktop/CARLA_0.9.16/RL_Agent/logs"
readonly WATCHDOG_LOG="${LOG_DIR}/system_watchdog.log"
readonly HEARTBEAT_FILE="/tmp/system_watchdog_heartbeat"
readonly MAX_NO_HEARTBEAT=300  # 5 minutes without heartbeat = frozen
readonly CHECK_INTERVAL=30      # Check every 30 seconds
readonly MAX_LOAD_AVG=50       # If load average > 50, system might be frozen
readonly MAX_IO_WAIT=90        # If I/O wait > 90%, system might be frozen

mkdir -p "${LOG_DIR}"

log_message() {
    local level="$1"
    shift
    local message="$*"
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo "${timestamp} | ${level} | ${message}" >> "${WATCHDOG_LOG}" 2>&1
    echo "[${timestamp}] ${level}: ${message}"
}

log_info() {
    log_message "INFO" "$@"
}

log_warning() {
    log_message "WARNING" "$@"
}

log_error() {
    log_message "ERROR" "$@"
}

# Update heartbeat file
update_heartbeat() {
    echo "$(date +%s)" > "${HEARTBEAT_FILE}" 2>/dev/null || true
}

# Check if system is responsive
check_system_responsive() {
    local start_time=$(date +%s)
    
    # Try to write to a file (test I/O)
    local test_file="/tmp/.watchdog_test_$$"
    if ! timeout 5 bash -c "echo test > ${test_file} 2>/dev/null && rm -f ${test_file}" 2>/dev/null; then
        log_warning "I/O test failed - system may be frozen"
        return 1
    fi
    
    # Check load average
    local load_avg=$(uptime | awk -F'load average:' '{print $2}' | awk '{print $1}' | tr -d ',')
    if [ -n "${load_avg}" ]; then
        local load_int=$(echo "${load_avg}" | cut -d. -f1)
        if [ "${load_int}" -gt "${MAX_LOAD_AVG}" ]; then
            log_warning "Load average too high: ${load_avg} (max: ${MAX_LOAD_AVG})"
            return 1
        fi
    fi
    
    # Check I/O wait (using top or iostat if available)
    local io_wait=$(top -bn1 | grep "%Cpu" | awk '{print $10}' | cut -d'%' -f1 | cut -d. -f1 2>/dev/null || echo "0")
    if [ -n "${io_wait}" ] && [ "${io_wait}" -gt "${MAX_IO_WAIT}" ]; then
        log_warning "I/O wait too high: ${io_wait}% (max: ${MAX_IO_WAIT}%)"
        return 1
    fi
    
    # Check if we can execute commands
    local end_time=$(date +%s)
    local elapsed=$((end_time - start_time))
    if [ "${elapsed}" -gt 10 ]; then
        log_warning "System response too slow: ${elapsed}s"
        return 1
    fi
    
    return 0
}

# Check if processes are stuck
check_processes_stuck() {
    # Check for too many zombie processes
    local zombie_count=$(ps aux | awk '$8 ~ /^[Zz]/ {count++} END {print count+0}')
    if [ "${zombie_count}" -gt 50 ]; then
        log_warning "Too many zombie processes: ${zombie_count}"
        return 1
    fi
    
    # Check if critical processes are responding
    local critical_processes=("systemd" "kernel")
    for proc in "${critical_processes[@]}"; do
        if ! pgrep -f "${proc}" > /dev/null 2>&1; then
            log_error "Critical process ${proc} not found!"
            return 1
        fi
    done
    
    return 0
}

# Check if system is frozen (no heartbeat updates)
check_heartbeat() {
    if [ ! -f "${HEARTBEAT_FILE}" ]; then
        log_warning "Heartbeat file not found, creating..."
        update_heartbeat
        return 0
    fi
    
    local last_heartbeat=$(cat "${HEARTBEAT_FILE}" 2>/dev/null || echo "0")
    local current_time=$(date +%s)
    local time_since_heartbeat=$((current_time - last_heartbeat))
    
    if [ "${time_since_heartbeat}" -gt "${MAX_NO_HEARTBEAT}" ]; then
        log_error "No heartbeat for ${time_since_heartbeat}s (max: ${MAX_NO_HEARTBEAT}s) - system may be frozen"
        return 1
    fi
    
    return 0
}

# Emergency reboot (with logging)
emergency_reboot() {
    local reason="$1"
    log_error "EMERGENCY REBOOT TRIGGERED: ${reason}"
    log_error "System will reboot in 10 seconds..."
    
    # Try to sync filesystem (with timeout)
    timeout 5 sync 2>/dev/null || true
    
    # Log to system log
    logger -t system_watchdog "EMERGENCY REBOOT: ${reason}"
    
    # Wait a bit for logs to flush
    sleep 2
    
    # Reboot
    /sbin/reboot
    exit 1
}

# Main watchdog loop
main() {
    log_info "System Watchdog started (PID: $$)"
    log_info "Max no-heartbeat: ${MAX_NO_HEARTBEAT}s, Check interval: ${CHECK_INTERVAL}s"
    
    local consecutive_failures=0
    local max_consecutive_failures=3  # Need 3 consecutive failures before reboot
    
    # Update heartbeat immediately
    update_heartbeat
    
    while true; do
        # Update heartbeat
        update_heartbeat
        
        # Check system responsiveness
        if ! check_system_responsive; then
            consecutive_failures=$((consecutive_failures + 1))
            log_warning "System responsiveness check failed (${consecutive_failures}/${max_consecutive_failures})"
        elif ! check_processes_stuck; then
            consecutive_failures=$((consecutive_failures + 1))
            log_warning "Process check failed (${consecutive_failures}/${max_consecutive_failures})"
        elif ! check_heartbeat; then
            consecutive_failures=$((consecutive_failures + 1))
            log_warning "Heartbeat check failed (${consecutive_failures}/${max_consecutive_failures})"
        else
            # System is healthy, reset failure counter
            if [ "${consecutive_failures}" -gt 0 ]; then
                log_info "System recovered, resetting failure counter"
                consecutive_failures=0
            fi
        fi
        
        # If we have too many consecutive failures, trigger emergency reboot
        if [ "${consecutive_failures}" -ge "${max_consecutive_failures}" ]; then
            emergency_reboot "System unresponsive (${consecutive_failures} consecutive failures)"
        fi
        
        # Sleep with timeout to prevent hanging
        timeout $((CHECK_INTERVAL + 5)) sleep "${CHECK_INTERVAL}" 2>/dev/null || {
            log_warning "Sleep interrupted, continuing..."
            sleep 1
        }
    done
}

# Handle signals
trap 'log_info "Watchdog stopped by signal"; exit 0' SIGTERM SIGINT

# Run main function
main "$@"

