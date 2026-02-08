#!/bin/bash
# Install PyTorch with ROCm support for AMD 7800XT

echo "🔧 Installing PyTorch with ROCm support for AMD GPU"
echo "=================================================="

# Check ROCm version
if [ -d "/opt/rocm" ]; then
    ROCM_VERSION=$(cat /opt/rocm/.info/version-* 2>/dev/null | head -1 || echo "5.7")
    echo "✅ ROCm found: $ROCM_VERSION"
else
    echo "⚠️  ROCm not found in /opt/rocm"
    echo "   Please install ROCm first: https://rocm.docs.amd.com/"
    exit 1
fi

# Determine ROCm version for PyTorch
ROCM_MAJOR=$(echo $ROCM_VERSION | cut -d. -f1)
ROCM_MINOR=$(echo $ROCM_VERSION | cut -d. -f2)

if [ "$ROCM_MAJOR" -eq 5 ] && [ "$ROCM_MINOR" -ge 7 ]; then
    ROCM_TORCH="rocm5.7"
elif [ "$ROCM_MAJOR" -eq 6 ]; then
    ROCM_TORCH="rocm6.0"
else
    ROCM_TORCH="rocm5.7"
    echo "⚠️  Using default ROCm 5.7 for PyTorch"
fi

echo "📦 Installing PyTorch with ROCm $ROCM_TORCH..."

# Uninstall existing torch if any
pip uninstall -y torch torchvision 2>/dev/null || true

# Install PyTorch with ROCm
pip install torch torchvision --index-url https://download.pytorch.org/whl/$ROCM_TORCH

# Verify installation
echo ""
echo "🔍 Verifying installation..."
python3 << EOF
import torch
print(f"PyTorch version: {torch.__version__}")
print(f"ROCm available: {hasattr(torch.version, 'hip') and torch.version.hip is not None}")
if torch.cuda.is_available():
    print(f"GPU: {torch.cuda.get_device_name(0)}")
    print(f"✅ ROCm GPU detected!")
else:
    print("⚠️  No GPU detected")
EOF

echo ""
echo "✅ Installation complete!"
echo ""
echo "To verify ROCm is working:"
echo "  python3 -c \"import torch; print('ROCm:', hasattr(torch.version, 'hip'))\""

