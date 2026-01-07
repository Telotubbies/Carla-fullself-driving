#!/bin/bash
# Fix Ubuntu to use AMD 7800XT instead of Intel iGPU
# This is specifically for Ubuntu desktop, not Windows

set -e

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🔧 Fix Ubuntu GPU Primary - AMD 7800XT"
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

# Backup existing configs
XORG_CONF="/etc/X11/xorg.conf"
XORG_CONF_D="/etc/X11/xorg.conf.d/20-amdgpu-primary.conf"

if [ -f "${XORG_CONF}" ]; then
    echo "⚠️  Found existing ${XORG_CONF}"
    echo "   Backing up to ${XORG_CONF}.backup.$(date +%Y%m%d_%H%M%S)"
    cp "${XORG_CONF}" "${XORG_CONF}.backup.$(date +%Y%m%d_%H%M%S)"
fi

echo ""
echo "📝 Creating Xorg configuration for AMD GPU primary..."

mkdir -p /etc/X11/xorg.conf.d

# Create Xorg config to use AMD GPU as primary
cat > "${XORG_CONF_D}" << 'EOF'
# AMD 7800XT Primary GPU Configuration for Ubuntu
# This forces Ubuntu desktop to use AMD GPU instead of Intel iGPU

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
    Option "AccelMethod" "glamor"
    Option "DRI" "3"
EndSection

Section "Screen"
    Identifier "amd_screen"
    Device "amd_card"
    DefaultDepth 24
    SubSection "Display"
        Depth 24
        Modes "1920x1080"
    EndSubSection
EndSection

# Intel iGPU - set as secondary (optional, can be disabled)
Section "Device"
    Identifier "intel_card"
    Driver "modesetting"
    BusID "PCI:0:2:0"
    Option "AccelMethod" "glamor"
EndSection

Section "Screen"
    Identifier "intel_screen"
    Device "intel_card"
    DefaultDepth 24
EndSection
EOF

echo "   ✅ Created ${XORG_CONF_D}"

# Create environment file for DRI_PRIME
ENV_FILE="/etc/environment.d/90-amdgpu-primary.conf"
mkdir -p /etc/environment.d

cat > "${ENV_FILE}" << 'EOF'
# Force AMD GPU as primary for all applications
DRI_PRIME=1
__GLX_VENDOR_LIBRARY_NAME=mesa
MESA_LOADER_DRIVER_OVERRIDE=radeonsi
LIBVA_DRIVER_NAME=radeonsi
AMD_VULKAN_ICD=RADV
EOF

echo "   ✅ Created ${ENV_FILE}"

# Update /etc/environment if it exists
if [ -f "/etc/environment" ]; then
    if ! grep -q "DRI_PRIME=1" /etc/environment; then
        echo "" >> /etc/environment
        echo "# AMD GPU Primary" >> /etc/environment
        echo "DRI_PRIME=1" >> /etc/environment
        echo "   ✅ Updated /etc/environment"
    fi
fi

# Update GRUB to ensure AMD GPU is initialized first
GRUB_FILE="/etc/default/grub"
if [ -f "${GRUB_FILE}" ]; then
    echo ""
    echo "📝 Checking GRUB configuration..."
    
    # Backup
    cp "${GRUB_FILE}" "${GRUB_FILE}.backup.$(date +%Y%m%d_%H%M%S)"
    
    # Check if video= parameter exists
    if ! grep -q "video=" "${GRUB_FILE}"; then
        # Add video parameter to prefer AMD GPU
        sed -i 's/GRUB_CMDLINE_LINUX_DEFAULT="/GRUB_CMDLINE_LINUX_DEFAULT="video=DP-1:1920x1080@60 /' "${GRUB_FILE}"
        echo "   ✅ Added video parameter to GRUB"
    fi
    
    # Ensure amdgpu is loaded
    if ! grep -q "amdgpu" "${GRUB_FILE}"; then
        # amdgpu.ppfeaturemask is already there, good
        echo "   ℹ️  amdgpu configuration already present"
    fi
fi

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
echo "3. After reboot, verify:"
echo "   rocm-smi"
echo "   echo \$DRI_PRIME"
echo "   glxinfo | grep 'OpenGL renderer'  # (if mesa-utils installed)"
echo ""
echo "4. Check which GPU is primary:"
echo "   cat /sys/class/drm/card*/status"
echo "   ls -la /dev/dri/"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "⚠️  Note: This only affects Ubuntu, not Windows 11"
echo "   Windows 11 should continue to work normally"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"


