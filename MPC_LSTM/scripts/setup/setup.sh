#!/bin/bash
# Setup script for CARLA LSTM-MPC Autonomous Driving System

set -e

echo "🚗 CARLA LSTM-MPC Setup Script"
echo "=============================="

# Get CARLA directory
CARLA_DIR="${CARLA_DIR:-/home/a/Desktop/CARLA_0.9.16}"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "CARLA Directory: $CARLA_DIR"
echo "Project Directory: $PROJECT_DIR"

# Check if CARLA exists
if [ ! -d "$CARLA_DIR" ]; then
    echo "❌ CARLA directory not found: $CARLA_DIR"
    echo "Please set CARLA_DIR environment variable or edit this script"
    exit 1
fi

# Find CARLA Python API egg
CARLA_EGG=$(find "$CARLA_DIR/PythonAPI/carla/dist" -name "carla-*.egg" 2>/dev/null | head -1)

if [ -z "$CARLA_EGG" ]; then
    echo "❌ CARLA Python API egg not found"
    echo "Please build CARLA Python API first:"
    echo "  cd $CARLA_DIR/PythonAPI/carla && make PythonAPI"
    exit 1
fi

echo "✅ Found CARLA Python API: $CARLA_EGG"

# Add to PYTHONPATH
export PYTHONPATH="$CARLA_EGG:$PYTHONPATH"

# Create .env file
ENV_FILE="$PROJECT_DIR/.env"
cat > "$ENV_FILE" << EOF
# CARLA Python API
export PYTHONPATH="$CARLA_EGG:\$PYTHONPATH"

# CARLA Server
export CARLA_HOST=localhost
export CARLA_PORT=2000
EOF

echo "✅ Created .env file: $ENV_FILE"

# Install Python dependencies
echo ""
echo "📦 Installing Python dependencies..."
pip install -r "$PROJECT_DIR/requirements.txt"

# Create necessary directories
echo ""
echo "📁 Creating directories..."
mkdir -p "$PROJECT_DIR/logs"
mkdir -p "$PROJECT_DIR/data"

echo ""
echo "✅ Setup complete!"
echo ""
echo "To use the system:"
echo "  1. Source the environment: source $ENV_FILE"
echo "  2. Start CARLA: cd $CARLA_DIR && ./CarlaUE4.sh"
echo "  3. Run the system: python $PROJECT_DIR/main.py --mode inference"

