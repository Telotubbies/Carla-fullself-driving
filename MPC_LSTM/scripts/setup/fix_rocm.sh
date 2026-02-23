#!/bin/bash
# Fix ROCm for AMD 7800XT (gfx1101 compatibility)

echo "🔧 Fixing ROCm for AMD 7800XT (gfx1101)"
echo "========================================"

# Set environment variable for gfx1101 -> gfx1100 compatibility
export HSA_OVERRIDE_GFX_VERSION=11.0.0

echo "✅ Set HSA_OVERRIDE_GFX_VERSION=11.0.0"
echo ""
echo "To make this permanent, add to ~/.bashrc:"
echo "  export HSA_OVERRIDE_GFX_VERSION=11.0.0"
echo ""
echo "Testing ROCm..."
python3 << 'EOF'
import torch
import os

# Set environment
os.environ['HSA_OVERRIDE_GFX_VERSION'] = '11.0.0'

print(f"PyTorch version: {torch.__version__}")
print(f"ROCm available: {hasattr(torch.version, 'hip') and torch.version.hip is not None}")

if torch.cuda.is_available():
    print(f"GPU: {torch.cuda.get_device_name(0)}")
    try:
        x = torch.randn(10, 10).cuda()
        result = x.sum().item()
        print(f"✅ GPU test passed: {result:.2f}")
    except Exception as e:
        print(f"❌ GPU test failed: {e}")
else:
    print("⚠️  No GPU detected")
EOF

echo ""
echo "✅ ROCm fix applied!"

