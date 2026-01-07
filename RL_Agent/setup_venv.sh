#!/bin/bash
# Setup Virtual Environment for RL Agent Training
# สำหรับ AMD GPU (RX 7800 XT) และ CARLA 0.9.16

set -e  # Exit on error

echo "=========================================="
echo "RL Agent Virtual Environment Setup"
echo "=========================================="

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Check Python version
echo "[1/6] Checking Python version..."
if ! command -v python3 &> /dev/null; then
    echo "❌ Python3 not found. Please install Python 3.8+"
    exit 1
fi

PYTHON_VERSION=$(python3 --version | cut -d' ' -f2 | cut -d'.' -f1,2)
echo "✅ Python version: $(python3 --version)"

# Create virtual environment
echo ""
echo "[2/6] Creating virtual environment..."
if [ -d "venv" ]; then
    echo "⚠️  Virtual environment already exists. Removing old one..."
    rm -rf venv
fi

python3 -m venv venv
echo "✅ Virtual environment created"

# Activate virtual environment
echo ""
echo "[3/6] Activating virtual environment..."
source venv/bin/activate
echo "✅ Virtual environment activated"

# Upgrade pip
echo ""
echo "[4/6] Upgrading pip..."
pip install --upgrade pip setuptools wheel
echo "✅ pip upgraded"

# Install CARLA Python API
echo ""
echo "[5/6] Installing CARLA Python API..."
CARLA_WHEEL="../PythonAPI/carla/dist/carla-0.9.16-*.whl"

# Find matching wheel for current Python version
PYTHON_MAJOR=$(python3 -c "import sys; print(sys.version_info.major)")
PYTHON_MINOR=$(python3 -c "import sys; print(sys.version_info.minor)")

if [ -f "../PythonAPI/carla/dist/carla-0.9.16-cp${PYTHON_MAJOR}${PYTHON_MINOR}-cp${PYTHON_MAJOR}${PYTHON_MINOR}-manylinux_2_31_x86_64.whl" ]; then
    CARLA_WHEEL="../PythonAPI/carla/dist/carla-0.9.16-cp${PYTHON_MAJOR}${PYTHON_MINOR}-cp${PYTHON_MAJOR}${PYTHON_MINOR}-manylinux_2_31_x86_64.whl"
    pip install "$CARLA_WHEEL"
    echo "✅ CARLA Python API installed"
else
    echo "⚠️  CARLA wheel not found for Python ${PYTHON_MAJOR}.${PYTHON_MINOR}"
    echo "   Available wheels:"
    ls -1 ../PythonAPI/carla/dist/*.whl 2>/dev/null || echo "   None found"
    echo "   Please install manually: pip install <wheel_file>"
fi

# Install requirements
echo ""
echo "[6/6] Installing Python dependencies..."
pip install -r requirements.txt
echo "✅ Dependencies installed"

# Verify GPU support
echo ""
echo "=========================================="
echo "Verifying GPU Support (AMD RX 7800 XT)"
echo "=========================================="
python utils/check_gpu.py

echo ""
echo "=========================================="
echo "✅ Setup Complete!"
echo "=========================================="
echo ""
echo "To activate the virtual environment:"
echo "  source venv/bin/activate"
echo ""
echo "To deactivate:"
echo "  deactivate"
echo ""
echo "To start training:"
echo "  1. Start CARLA: cd .. && ./CarlaUE4.sh"
echo "  2. Train agent: python training/train.py --config config/phase1_config.yaml"
echo ""

