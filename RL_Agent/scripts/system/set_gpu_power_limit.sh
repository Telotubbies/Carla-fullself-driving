#!/bin/bash
# Set GPU Power Limit for RX 7800 XT to 100% (full power)
# RX 7800 XT TDP: 263W, can spike to 350W+

GPU_CARD="/sys/class/drm/card1/device"
POWER_LIMIT_PERCENT=100  # Full power (100%)

if [ -d "$GPU_CARD" ]; then
    # Set performance level to high (not auto/manual to prevent spikes)
    if [ -f "${GPU_CARD}/power_dpm_force_performance_level" ]; then
        echo "high" > "${GPU_CARD}/power_dpm_force_performance_level" 2>/dev/null
        echo "GPU power level set to: high"
    fi
    
    # Set power profile to performance (full power)
    if [ -f "${GPU_CARD}/pp_power_profile_mode" ]; then
        echo 2 > "${GPU_CARD}/pp_power_profile_mode" 2>/dev/null || true  # 2 = performance mode
        echo "GPU power profile set to: performance (100%)"
    fi
    
    # Try to set power cap to 100% if available
    for hwmon in ${GPU_CARD}/hwmon/hwmon*; do
        if [ -f "${hwmon}/power1_cap_max" ]; then
            MAX_POWER=$(cat "${hwmon}/power1_cap_max" 2>/dev/null)
            if [ -n "$MAX_POWER" ] && [ "$MAX_POWER" -gt 0 ]; then
                # Set to 100% (full power)
                if [ -f "${hwmon}/power1_cap" ]; then
                    echo "$MAX_POWER" > "${hwmon}/power1_cap" 2>/dev/null && \
                    echo "GPU power limit set to ${MAX_POWER}W (100% of max)" || \
                    echo "Could not set power limit (may require root)"
                fi
            fi
        fi
    done
else
    echo "GPU device not found at $GPU_CARD"
fi
