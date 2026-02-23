#!/bin/bash
# Single script to run everything - Setup + CARLA + Main System
# สำหรับ ROCm (AMD 7800XT) และ PostgreSQL (optional)

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get directories
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CARLA_DIR="${CARLA_DIR:-/home/a/Desktop/CARLA_0.9.16}"
PROJECT_DIR="$SCRIPT_DIR"

echo -e "${BLUE}🚗 CARLA LSTM-MPC Autonomous Driving - All-in-One Script${NC}"
echo -e "${BLUE}========================================================${NC}"
echo ""

# Function to check if CARLA is running
check_carla_running() {
    if pgrep -f "CarlaUE4" > /dev/null; then
        return 0
    else
        return 1
    fi
}

# Function to wait for CARLA
wait_for_carla() {
    echo -e "${YELLOW}⏳ Waiting for CARLA to be ready...${NC}"
    max_attempts=30
    attempt=0
    
    while [ $attempt -lt $max_attempts ]; do
        if timeout 2 python3 -c "import carla; client = carla.Client('localhost', 2000); client.set_timeout(2.0); client.get_world()" 2>/dev/null; then
            echo -e "${GREEN}✅ CARLA is ready!${NC}"
            return 0
        fi
        attempt=$((attempt + 1))
        sleep 2
    done
    
    echo -e "${RED}❌ CARLA did not become ready in time${NC}"
    return 1
}

# Step 1: Check dependencies
echo -e "${BLUE}📦 Step 1: Checking dependencies...${NC}"

# Check Python
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}❌ Python3 not found${NC}"
    exit 1
fi
echo -e "${GREEN}✅ Python3 found${NC}"

# Check if in virtual environment (optional)
if [ -z "$VIRTUAL_ENV" ]; then
    echo -e "${YELLOW}⚠️  Not in virtual environment (recommended)${NC}"
fi

# Step 2: Setup environment
echo ""
echo -e "${BLUE}🔧 Step 2: Setting up environment...${NC}"

# Find CARLA Python API
CARLA_EGG=$(find "$CARLA_DIR/PythonAPI/carla/dist" -name "carla-*.egg" 2>/dev/null | head -1)

if [ -z "$CARLA_EGG" ]; then
    echo -e "${RED}❌ CARLA Python API not found${NC}"
    echo "Please build CARLA Python API first:"
    echo "  cd $CARLA_DIR/PythonAPI/carla && make PythonAPI"
    exit 1
fi

echo -e "${GREEN}✅ Found CARLA Python API: $CARLA_EGG${NC}"

# Add to PYTHONPATH
export PYTHONPATH="$CARLA_EGG:$PYTHONPATH"
echo -e "${GREEN}✅ PYTHONPATH updated${NC}"

# Fix ROCm for AMD 7800XT (gfx1101)
if command -v rocm-smi &> /dev/null; then
    export HSA_OVERRIDE_GFX_VERSION=11.0.0
    echo -e "${GREEN}✅ ROCm environment configured for AMD 7800XT${NC}"
fi

# Check if requirements are installed
echo -e "${YELLOW}Checking Python packages...${NC}"
if ! python3 -c "import torch; import carla; import casadi; import pygame" 2>/dev/null; then
    echo -e "${YELLOW}⚠️  Some packages missing. Installing...${NC}"
    pip install -q -r "$PROJECT_DIR/requirements.txt"
    echo -e "${GREEN}✅ Packages installed${NC}"
else
    echo -e "${GREEN}✅ Required packages available${NC}"
fi

# Check ROCm
echo -e "${YELLOW}Checking GPU support...${NC}"
if python3 -c "import torch; print('ROCm:', hasattr(torch.version, 'hip') and torch.version.hip is not None); print('CUDA:', torch.cuda.is_available())" 2>/dev/null; then
    python3 -c "import torch; rocm = hasattr(torch.version, 'hip') and torch.version.hip is not None; cuda = torch.cuda.is_available() and not rocm; print('✅ ROCm available' if rocm else ('✅ CUDA available' if cuda else '⚠️  Using CPU'))"
fi

# Check PostgreSQL (optional)
if python3 -c "import psycopg2" 2>/dev/null; then
    echo -e "${GREEN}✅ PostgreSQL libraries available${NC}"
else
    echo -e "${YELLOW}⚠️  PostgreSQL libraries not installed (optional)${NC}"
    echo "   Install with: pip install psycopg2-binary SQLAlchemy"
fi

# Step 3: Start CARLA
echo ""
echo -e "${BLUE}🎮 Step 3: Starting CARLA Simulator...${NC}"

if check_carla_running; then
    echo -e "${GREEN}✅ CARLA is already running${NC}"
else
    echo -e "${YELLOW}Starting CARLA server...${NC}"
    cd "$CARLA_DIR"
    
    # Start CARLA in background
    nohup ./CarlaUE4.sh > /tmp/carla.log 2>&1 &
    CARLA_PID=$!
    echo -e "${GREEN}✅ CARLA started (PID: $CARLA_PID)${NC}"
    echo -e "${YELLOW}   Logs: /tmp/carla.log${NC}"
    
    # Wait for CARLA to be ready
    if ! wait_for_carla; then
        echo -e "${RED}❌ Failed to start CARLA${NC}"
        kill $CARLA_PID 2>/dev/null || true
        exit 1
    fi
fi

# Step 4: Parse arguments
MODE="${1:-inference}"
CONFIG="${2:-config.yaml}"

echo ""
echo -e "${BLUE}🚀 Step 4: Running system in ${MODE} mode...${NC}"
echo ""

# Step 5: Run main system
cd "$PROJECT_DIR"

# Create necessary directories
mkdir -p logs data

# Run the system
echo -e "${GREEN}Starting autonomous driving system...${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop${NC}"
echo ""

# Run with ROCm fix if needed
python3 main.py --mode "$MODE" --config "$CONFIG" || EXIT_CODE=$?

# Cleanup
echo ""
echo -e "${BLUE}🧹 Cleaning up...${NC}"

# Optionally kill CARLA if we started it
if [ ! -z "$CARLA_PID" ]; then
    echo -e "${YELLOW}Stopping CARLA (PID: $CARLA_PID)...${NC}"
    kill $CARLA_PID 2>/dev/null || true
    sleep 2
    kill -9 $CARLA_PID 2>/dev/null || true
fi

if [ ${EXIT_CODE:-0} -eq 0 ]; then
    echo -e "${GREEN}✅ System completed successfully${NC}"
    exit 0
else
    echo -e "${RED}❌ System exited with error${NC}"
    exit ${EXIT_CODE:-1}
fi

