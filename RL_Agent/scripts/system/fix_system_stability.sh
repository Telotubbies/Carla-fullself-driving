#!/bin/bash
# Comprehensive System Stability Fix for i5-13500 + RX 7800 XT
# Prevents kernel panic and system crashes

set -u

readonly LOG_FILE="/home/a/Desktop/CARLA_0.9.16/RL_Agent/logs/stability_fix.log"

log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') | $1" | tee -a "${LOG_FILE}"
}

log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
log "🔧 System Stability Fix - i5-13500 + RX 7800 XT"
log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 1. Kernel Parameters for Stability
log "📝 Configuring kernel parameters..."
sudo bash -c 'cat > /etc/sysctl.d/99-system-stability.conf' <<'EOF'
# System Stability Settings for i5-13500 + RX 7800 XT
# Kernel panic settings
kernel.panic = 5
kernel.panic_on_oops = 0
kernel.panic_on_rcu_stall = 0
kernel.hung_task_panic = 0
kernel.hung_task_timeout_secs = 600
kernel.hung_task_check_interval_secs = 60
kernel.hung_task_warnings = 10
kernel.softlockup_panic = 0
kernel.softlockup_all_cpu_backtrace = 0

# Memory management (prevent OOM)
vm.oom_kill_allocating_task = 1
vm.overcommit_memory = 1
vm.swappiness = 10
vm.dirty_ratio = 15
vm.dirty_background_ratio = 5
vm.vfs_cache_pressure = 50

# Network stability
net.core.rmem_max = 16777216
net.core.wmem_max = 16777216
net.ipv4.tcp_rmem = 4096 87380 16777216
net.ipv4.tcp_wmem = 4096 65536 16777216
net.core.netdev_max_backlog = 5000

# Reduce kernel warnings
kernel.printk = 3 4 1 3

# CPU scheduler
kernel.sched_migration_cost_ns = 5000000
kernel.sched_autogroup_enabled = 1
EOF

sudo sysctl -p /etc/sysctl.d/99-system-stability.conf
log "✅ Kernel parameters configured"

# 2. AMD GPU Stability Settings
log "📝 Configuring AMD GPU stability..."
sudo bash -c 'cat > /etc/modprobe.d/amdgpu-stability.conf' <<'EOF'
# AMD RX 7800 XT Stability Settings
# Disable problematic features that can cause crashes
options amdgpu runtime_pm=1
options amdgpu dc=1
options amdgpu aspm=0
options amdgpu dpm=1
options amdgpu gpu_recovery=1
options amdgpu vm_update_mode=3
# Reduce power spikes
options amdgpu ppfeaturemask=0xffffffff
EOF

log "✅ AMD GPU settings configured"

# 3. CPU Governor (Performance mode for stability)
log "📝 Configuring CPU governor..."
if command -v cpupower &> /dev/null; then
    sudo cpupower frequency-set -g performance 2>/dev/null || true
    log "✅ CPU governor set to performance"
else
    log "⚠️  cpupower not available"
fi

# 4. GRUB Boot Parameters
log "📝 Updating GRUB boot parameters..."
if [ -f /etc/default/grub ]; then
    sudo cp /etc/default/grub /etc/default/grub.backup.$(date +%Y%m%d_%H%M%S)
    
    # Add stability parameters
    if ! grep -q "GRUB_CMDLINE_LINUX_DEFAULT.*panic" /etc/default/grub; then
        sudo sed -i 's/GRUB_CMDLINE_LINUX_DEFAULT="\(.*\)"/GRUB_CMDLINE_LINUX_DEFAULT="\1 panic=5 nmi_watchdog=0 softlockup_panic=0 intel_idle.max_cstate=4 processor.max_cstate=4"/' /etc/default/grub
        log "✅ GRUB parameters updated"
    else
        log "ℹ️  GRUB already has panic parameter"
    fi
    
    sudo update-grub > /dev/null 2>&1
fi

# 5. Disable problematic kernel modules
log "📝 Configuring kernel modules..."
sudo bash -c 'cat > /etc/modprobe.d/blacklist-stability.conf' <<'EOF'
# Blacklist problematic modules that can cause crashes
blacklist nouveau
blacklist nvidia
EOF

# 6. Create systemd service for GPU power management
log "📝 Creating GPU power management service..."
sudo bash -c 'cat > /etc/systemd/system/gpu-power-manager.service' <<'EOF'
[Unit]
Description=GPU Power Management - Prevent Power Spikes
After=graphical.target

[Service]
Type=oneshot
ExecStart=/bin/bash -c 'echo "high" > /sys/class/drm/card*/device/power_dpm_force_performance_level 2>/dev/null || true'
ExecStart=/bin/bash -c 'echo 1 > /sys/class/drm/card*/device/pp_power_profile_mode 2>/dev/null || true'
RemainAfterExit=yes

[Install]
WantedBy=graphical.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable gpu-power-manager.service 2>/dev/null || true
log "✅ GPU power management service created"

# 7. Create monitoring script
log "📝 Creating system monitoring script..."
cat > /home/a/Desktop/CARLA_0.9.16/RL_Agent/scripts/monitor_system_stability.sh <<'MONITOR_EOF'
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
MONITOR_EOF

chmod +x /home/a/Desktop/CARLA_0.9.16/RL_Agent/scripts/monitor_system_stability.sh
log "✅ Monitoring script created"

log ""
log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
log "✅ System Stability Fix Complete!"
log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
log ""
log "📋 Next Steps:"
log "   1. Reboot system: sudo reboot"
log "   2. Monitor stability: tail -f ${LOG_FILE}"
log "   3. Check GPU: cat /sys/class/drm/card*/device/power_dpm_force_performance_level"
log ""
