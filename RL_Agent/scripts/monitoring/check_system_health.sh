#!/bin/bash
# Comprehensive System Health Check

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🔍 System Health Check - i5-13500 + RX 7800 XT"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

echo "📊 CPU Status:"
echo "   Model: $(lscpu | grep 'Model name' | cut -d: -f2 | xargs)"
echo "   Cores: $(nproc)"
echo "   Load: $(uptime | awk -F'load average:' '{print $2}')"
echo "   Governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo 'N/A')"
echo ""

echo "🎮 GPU Status:"
GPU_INFO=$(lspci | grep -i vga)
echo "   $GPU_INFO"
if [ -f /sys/class/drm/card0/device/power_dpm_force_performance_level ]; then
    echo "   Power Level: $(cat /sys/class/drm/card0/device/power_dpm_force_performance_level 2>/dev/null)"
fi
echo ""

echo "🌡️  Temperature:"
sensors 2>/dev/null | grep -E "edge|junction|Package|Core" | head -5
echo ""

echo "💾 Memory:"
free -h | grep -E "Mem|Swap"
echo ""

echo "⚡ Power & Stability:"
echo "   Kernel Panic: $(sysctl -n kernel.panic 2>/dev/null || echo 'N/A')"
echo "   Panic on Oops: $(sysctl -n kernel.panic_on_oops 2>/dev/null || echo 'N/A')"
echo "   Uptime: $(uptime -p)"
echo ""

echo "📋 Recent Reboots:"
last reboot | head -5
echo ""

echo "⚠️  Recent Errors (last 1 hour):"
journalctl -p err --since "1 hour ago" 2>/dev/null | tail -5 || echo "   ✅ No errors"
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
