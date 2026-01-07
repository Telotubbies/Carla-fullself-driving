#!/bin/bash
# Monitor system stability and log issues

LOG_DIR="/home/a/Desktop/CARLA_0.9.16/RL_Agent/logs"
LOG_FILE="${LOG_DIR}/system_stability_monitor.log"

check_temperature() {
    local temp=$(sensors 2>/dev/null | grep -E "edge|junction" | head -1 | grep -oE '[0-9]+\.[0-9]+' | head -1)
    if [ -n "$temp" ]; then
        if (( $(echo "$temp > 90" | bc -l) )); then
            echo "$(date): WARNING: GPU temperature high: ${temp}°C" >> "$LOG_FILE"
        fi
    fi
}

check_memory() {
    local mem_usage=$(free | grep Mem | awk '{printf "%.1f", $3/$2 * 100}')
    if (( $(echo "$mem_usage > 95" | bc -l) )); then
        echo "$(date): WARNING: Memory usage high: ${mem_usage}%" >> "$LOG_FILE"
    fi
}

check_kernel_errors() {
    local errors=$(journalctl -k --since "5 minutes ago" | grep -iE "error|panic|oops" | wc -l)
    if [ "$errors" -gt 0 ]; then
        echo "$(date): WARNING: Found $errors kernel errors in last 5 minutes" >> "$LOG_FILE"
    fi
}

while true; do
    check_temperature
    check_memory
    check_kernel_errors
    sleep 300  # Check every 5 minutes
done
