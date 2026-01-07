#!/bin/bash
# Script to configure system to use AMD 7800XT as primary GPU instead of Intel iGPU

set -e

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🔧 GPU Primary Configuration - AMD 7800XT"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "❌ Please run as root (use sudo)"
    exit 1
fi

# Detect GPUs
INTEL_GPU=$(lspci | grep -i "intel.*display" | head -1)
AMD_GPU=$(lspci | grep -i "amd.*radeon.*7800" | head -1)

echo "📊 Detected GPUs:"
echo "   Intel iGPU: ${INTEL_GPU:-Not found}"
echo "   AMD 7800XT: ${AMD_GPU:-Not found}"
echo ""

if [ -z "${AMD_GPU}" ]; then
    echo "❌ AMD 7800XT not found!"
    exit 1
fi

# Check current X server
XORG_CONF="/etc/X11/xorg.conf"
XORG_CONF_D="/etc/X11/xorg.conf.d/20-amdgpu.conf"

echo "🔍 Checking current configuration..."
if [ -f "${XORG_CONF}" ]; then
    echo "   ⚠️  Found existing ${XORG_CONF}"
    echo "   Backing up to ${XORG_CONF}.backup"
    cp "${XORG_CONF}" "${XORG_CONF}.backup"
fi

# Create Xorg configuration to use AMD GPU as primary
echo ""
echo "📝 Creating Xorg configuration..."

mkdir -p /etc/X11/xorg.conf.d

cat > "${XORG_CONF_D}" << 'EOF'
# AMD 7800XT Primary GPU Configuration
# This file forces the system to use AMD GPU as primary display

Section "ServerLayout"
    Identifier "layout"
    Screen 0 "amd_screen"
    Option "AutoAddGPU" "off"
EndSection

Section "Device"
    Identifier "amd_card"
    Driver "amdgpu"
    BusID "PCI:3:0:0"
    Option "PrimaryGPU" "yes"
EndSection

Section "Screen"
    Identifier "amd_screen"
    Device "amd_card"
    DefaultDepth 24
    SubSection "Display"
        Depth 24
    EndSubSection
EndSection

# Disable Intel iGPU (optional - comment out if you want both)
# Section "Device"
#     Identifier "intel_card"
#     Driver "modesetting"
#     BusID "PCI:0:2:0"
#     Option "Ignore" "true"
# EndSection
EOF

echo "   ✅ Created ${XORG_CONF_D}"

# Update GRUB to disable iGPU in kernel (optional)
GRUB_FILE="/etc/default/grub"
if [ -f "${GRUB_FILE}" ]; then
    echo ""
    echo "📝 Updating GRUB configuration..."
    
    # Backup
    cp "${GRUB_FILE}" "${GRUB_FILE}.backup.$(date +%Y%m%d_%H%M%S)"
    
    # Check if i915.modeset=0 is already set
    if ! grep -q "i915.modeset=0" "${GRUB_FILE}"; then
        # Add i915.modeset=0 to disable Intel iGPU
        sed -i 's/GRUB_CMDLINE_LINUX_DEFAULT="/GRUB_CMDLINE_LINUX_DEFAULT="i915.modeset=0 /' "${GRUB_FILE}"
        echo "   ✅ Added i915.modeset=0 to GRUB"
        echo "   ⚠️  Run 'sudo update-grub' to apply changes"
    else
        echo "   ℹ️  i915.modeset=0 already set in GRUB"
    fi
fi

# Create environment configuration
ENV_FILE="/etc/environment.d/90-amdgpu.conf"
mkdir -p /etc/environment.d

cat > "${ENV_FILE}" << 'EOF'
# Force AMD GPU as primary
DRI_PRIME=1
__GLX_VENDOR_LIBRARY_NAME=mesa
MESA_LOADER_DRIVER_OVERRIDE=radeonsi
EOF

echo "   ✅ Created ${ENV_FILE}"

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✅ Configuration Complete!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📋 Next Steps:"
echo ""
echo "1. Update GRUB (if changed):"
echo "   sudo update-grub"
echo ""
echo "2. Reboot the system:"
echo "   sudo reboot"
echo ""
echo "3. After reboot, verify GPU:"
echo "   rocm-smi"
echo "   glxinfo | grep 'OpenGL renderer'"
echo ""
echo "4. If display doesn't work, connect monitor to AMD GPU ports:"
echo "   - Check card1-DP-1, card1-DP-2, card1-DP-3, or card1-HDMI-A-1"
echo ""
echo "5. If issues persist, restore backup:"
echo "   sudo cp ${XORG_CONF_D}.backup ${XORG_CONF_D}"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"


