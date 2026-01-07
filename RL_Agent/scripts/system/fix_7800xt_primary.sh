#!/bin/bash
# Fix AMD 7800XT to be primary GPU instead of Intel iGPU
# This script addresses amdgpu probe failed error -22

set -e

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🔧 Fix AMD 7800XT Primary GPU - Complete Solution"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "❌ Please run as root (use sudo)"
    exit 1
fi

# Detect GPUs
INTEL_GPU=$(lspci | grep -i "intel.*display" | head -1)
AMD_GPU=$(lspci | grep -i "amd.*radeon.*7800\|navi.*32" | head -1)
AMD_PCI_ID=$(lspci -nn | grep -i "7800\|navi.*32" | grep -oP "\[1002:\K[0-9a-f]{4}")

echo "📊 Detected GPUs:"
echo "   Intel iGPU: ${INTEL_GPU:-Not found}"
echo "   AMD 7800XT: ${AMD_GPU:-Not found}"
echo "   AMD PCI ID: ${AMD_PCI_ID:-Not found}"
echo ""

if [ -z "${AMD_GPU}" ]; then
    echo "❌ AMD 7800XT not found!"
    exit 1
fi

# Check amdgpu module status
echo "🔍 Checking amdgpu driver status..."
if dmesg | grep -q "amdgpu.*probe.*failed"; then
    echo "   ⚠️  amdgpu probe failed detected"
    PROBE_ERROR=$(dmesg | grep "amdgpu.*probe.*failed" | tail -1)
    echo "   Error: ${PROBE_ERROR}"
fi

# Step 1: Update firmware (if needed)
echo ""
echo "📦 Step 1: Checking firmware..."
FIRMWARE_DIR="/lib/firmware/amdgpu"
if [ ! -d "${FIRMWARE_DIR}" ]; then
    echo "   ⚠️  Firmware directory not found"
else
    # Check for Navi 32 firmware
    NAVI32_FW=$(find "${FIRMWARE_DIR}" -name "*navi32*" 2>/dev/null | wc -l)
    if [ "${NAVI32_FW}" -eq 0 ]; then
        echo "   ⚠️  Navi 32 firmware not found"
        echo "   💡 Installing latest firmware..."
        
        # Try to update firmware
        apt-get update -qq
        apt-get install -y linux-firmware 2>/dev/null || echo "   ⚠️  Could not update firmware automatically"
        
        # Check if firmware package exists
        if [ -f "/usr/lib/firmware/amdgpu/navi32_*.bin" ] 2>/dev/null; then
            echo "   ✅ Firmware package installed"
        else
            echo "   ⚠️  Firmware may need manual download from kernel.org"
        fi
    else
        echo "   ✅ Navi 32 firmware found (${NAVI32_FW} files)"
    fi
fi

# Step 2: Blacklist Intel iGPU module
echo ""
echo "📝 Step 2: Configuring module blacklist..."
BLACKLIST_FILE="/etc/modprobe.d/blacklist-intel-igpu.conf"
cat > "${BLACKLIST_FILE}" << 'EOF'
# Blacklist Intel iGPU to force AMD GPU as primary
blacklist i915
options i915 modeset=0
EOF
echo "   ✅ Created ${BLACKLIST_FILE}"

# Step 3: Force amdgpu to load early
echo ""
echo "📝 Step 3: Configuring amdgpu module..."
MODPROBE_FILE="/etc/modprobe.d/amdgpu.conf"
cat > "${MODPROBE_FILE}" << 'EOF'
# Force amdgpu to load and work properly
options amdgpu si_support=1 cik_support=1
options amdgpu dc=1
EOF
echo "   ✅ Created ${MODPROBE_FILE}"

# Step 4: Update GRUB to disable iGPU and force AMD GPU
echo ""
echo "📝 Step 4: Updating GRUB configuration..."
GRUB_FILE="/etc/default/grub"
if [ -f "${GRUB_FILE}" ]; then
    # Backup
    cp "${GRUB_FILE}" "${GRUB_FILE}.backup.$(date +%Y%m%d_%H%M%S)"
    
    # Get current GRUB_CMDLINE_LINUX_DEFAULT
    CURRENT_CMDLINE=$(grep "^GRUB_CMDLINE_LINUX_DEFAULT=" "${GRUB_FILE}" | cut -d'"' -f2)
    
    # Add parameters if not present
    NEW_CMDLINE="${CURRENT_CMDLINE}"
    
    # Disable Intel iGPU
    if [[ ! "${NEW_CMDLINE}" =~ i915\.modeset=0 ]]; then
        NEW_CMDLINE="${NEW_CMDLINE} i915.modeset=0"
    fi
    
    # Force amdgpu
    if [[ ! "${NEW_CMDLINE}" =~ amdgpu\.si_support ]]; then
        NEW_CMDLINE="${NEW_CMDLINE} amdgpu.si_support=1 amdgpu.cik_support=1"
    fi
    
    # Video output to PCIe (AMD GPU)
    if [[ ! "${NEW_CMDLINE}" =~ video= ]]; then
        NEW_CMDLINE="${NEW_CMDLINE} video=DP-1:1920x1080@60"
    fi
    
    # Update GRUB
    sed -i "s|^GRUB_CMDLINE_LINUX_DEFAULT=.*|GRUB_CMDLINE_LINUX_DEFAULT=\"${NEW_CMDLINE}\"|" "${GRUB_FILE}"
    
    echo "   ✅ Updated GRUB_CMDLINE_LINUX_DEFAULT"
    echo "   New parameters: ${NEW_CMDLINE}"
else
    echo "   ⚠️  GRUB file not found"
fi

# Step 5: Create Xorg configuration
echo ""
echo "📝 Step 5: Creating Xorg configuration..."
mkdir -p /etc/X11/xorg.conf.d

XORG_CONF="/etc/X11/xorg.conf.d/20-amdgpu-primary.conf"
cat > "${XORG_CONF}" << 'EOF'
# AMD 7800XT Primary GPU Configuration
# This forces the system to use AMD GPU as primary display

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
    Option "TearFree" "true"
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

# Disable Intel iGPU
Section "Device"
    Identifier "intel_card"
    Driver "modesetting"
    BusID "PCI:0:2:0"
    Option "Ignore" "true"
EndSection
EOF

echo "   ✅ Created ${XORG_CONF}"

# Step 6: Environment variables
echo ""
echo "📝 Step 6: Setting environment variables..."
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

# Also update /etc/environment
if [ -f "/etc/environment" ]; then
    if ! grep -q "DRI_PRIME=1" /etc/environment; then
        echo "" >> /etc/environment
        echo "# AMD GPU Primary" >> /etc/environment
        echo "DRI_PRIME=1" >> /etc/environment
        echo "   ✅ Updated /etc/environment"
    fi
fi

# Step 7: Update initramfs
echo ""
echo "📝 Step 7: Updating initramfs..."
update-initramfs -u -k all 2>/dev/null || echo "   ⚠️  Could not update initramfs"

# Step 8: Update GRUB
echo ""
echo "📝 Step 8: Updating GRUB..."
update-grub 2>/dev/null || echo "   ⚠️  Could not update GRUB"

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✅ Configuration Complete!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📋 IMPORTANT: BIOS/UEFI Settings Required!"
echo ""
echo "⚠️  You MUST also configure BIOS/UEFI:"
echo ""
echo "1. Restart and enter BIOS/UEFI (usually F2, Del, or F10)"
echo "2. Navigate to: Advanced → Chipset Configuration"
echo "3. Set 'Primary Display' or 'Initiate Graphics Adapter' to:"
echo "   - 'PCIe' or 'Discrete Graphics' or 'PEG'"
echo "4. Set 'Integrated Graphics' or 'iGPU' to:"
echo "   - 'Disabled' or 'Auto'"
echo "5. Save and Exit"
echo ""
echo "📋 Next Steps:"
echo ""
echo "1. Reboot the system:"
echo "   sudo reboot"
echo ""
echo "2. After reboot, verify GPU:"
echo "   lspci | grep -i vga"
echo "   dmesg | grep -i amdgpu"
echo "   lsmod | grep amdgpu"
echo ""
echo "3. Check if amdgpu is working:"
echo "   ls -la /sys/class/drm/ | grep card"
echo "   cat /sys/class/drm/card*/device/uevent | grep DRIVER"
echo ""
echo "4. If display doesn't work:"
echo "   - Connect monitor to AMD GPU ports (not motherboard)"
echo "   - HDMI or DisplayPort on the GPU card"
echo ""
echo "5. If amdgpu still fails:"
echo "   - Check dmesg: sudo dmesg | grep amdgpu"
echo "   - May need newer kernel (6.15+) or firmware update"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"


