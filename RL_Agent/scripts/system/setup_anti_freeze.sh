#!/bin/bash
# Setup Anti-Freeze System - Configures system to prevent freezes and auto-reboot

set -u

readonly SCRIPT_DIR="/home/a/Desktop/CARLA_0.9.16/RL_Agent/scripts"
readonly LOG_FILE="/tmp/setup_anti_freeze.log"

log_message() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') | $1" | tee -a "${LOG_FILE}"
}

log_message "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
log_message "🔧 Setting up Anti-Freeze System"
log_message "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 1. Configure kernel panic settings
log_message "📝 Configuring kernel panic settings..."
sudo bash -c 'cat > /etc/sysctl.d/99-anti-freeze.conf' <<'EOF'
# Anti-Freeze Kernel Settings
kernel.panic = 10
kernel.panic_on_oops = 1
kernel.hung_task_panic = 1
kernel.hung_task_timeout_secs = 300
kernel.hung_task_check_interval_secs = 30
kernel.hung_task_warnings = 5
vm.oom_kill_allocating_task = 1
vm.overcommit_memory = 1
EOF

sudo sysctl -p /etc/sysctl.d/99-anti-freeze.conf
log_message "✅ Kernel panic settings configured"

# 2. Create systemd service for system watchdog
log_message "📝 Creating systemd service for system watchdog..."
sudo bash -c "cat > /etc/systemd/system/system-watchdog.service" <<EOF
[Unit]
Description=System Watchdog - Prevents system freezes
After=network.target

[Service]
Type=simple
User=a
Group=a
ExecStart=/bin/bash ${SCRIPT_DIR}/system_watchdog.sh
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
log_message "✅ System watchdog service created"

# 3. Create systemd service for auto_manage
log_message "📝 Creating systemd service for auto_manage..."
sudo bash -c "cat > /etc/systemd/system/auto-manage.service" <<EOF
[Unit]
Description=CARLA Auto Management Service
After=network.target

[Service]
Type=simple
User=a
Group=a
WorkingDirectory=${SCRIPT_DIR}/..
ExecStart=/bin/bash ${SCRIPT_DIR}/auto_manage.sh start
Restart=always
RestartSec=30
StandardOutput=journal
StandardError=journal
Environment="DISPLAY=:0"
Environment="XAUTHORITY=/home/a/.Xauthority"

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
log_message "✅ Auto-manage service created"

# 4. Make scripts executable
log_message "📝 Making scripts executable..."
chmod +x "${SCRIPT_DIR}/system_watchdog.sh"
chmod +x "${SCRIPT_DIR}/auto_manage.sh"
log_message "✅ Scripts made executable"

# 5. Configure GRUB for kernel panic reboot
log_message "📝 Configuring GRUB for kernel panic auto-reboot..."
if [ -f /etc/default/grub ]; then
    if ! grep -q "GRUB_CMDLINE_LINUX.*panic" /etc/default/grub; then
        sudo sed -i 's/GRUB_CMDLINE_LINUX_DEFAULT="\(.*\)"/GRUB_CMDLINE_LINUX_DEFAULT="\1 panic=10"/' /etc/default/grub
        log_message "✅ GRUB updated with panic=10"
    else
        log_message "ℹ️  GRUB already has panic parameter"
    fi
else
    log_message "⚠️  /etc/default/grub not found, skipping GRUB update"
fi

# 6. Summary
log_message ""
log_message "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
log_message "✅ Anti-Freeze System Setup Complete!"
log_message "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
log_message ""
log_message "📋 Next Steps:"
log_message ""
log_message "1. Enable and start services:"
log_message "   sudo systemctl enable system-watchdog.service"
log_message "   sudo systemctl start system-watchdog.service"
log_message "   sudo systemctl enable auto-manage.service"
log_message "   sudo systemctl start auto-manage.service"
log_message ""
log_message "2. Update GRUB (if changed):"
log_message "   sudo update-grub"
log_message ""
log_message "3. Reboot to apply kernel settings:"
log_message "   sudo reboot"
log_message ""
log_message "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

