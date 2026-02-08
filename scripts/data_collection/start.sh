#!/bin/bash
# 🚗 Quick Start Script - รันทุกอย่างพร้อมกัน

# Fix ROCm for AMD 7800XT
export HSA_OVERRIDE_GFX_VERSION=11.0.0

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}🚗 CARLA LSTM-MPC - Quick Start${NC}"
echo "================================"
echo ""

# Check if CARLA is running
if ! pgrep -f "CarlaUE4" > /dev/null; then
    echo -e "${YELLOW}⚠️  CARLA not running. Starting...${NC}"
    CARLA_DIR="${CARLA_DIR:-/home/a/Desktop/CARLA_0.9.16}"
    cd "$CARLA_DIR"
    nohup ./CarlaUE4.sh > /tmp/carla.log 2>&1 &
    echo -e "${GREEN}✅ CARLA starting (check /tmp/carla.log)${NC}"
    echo -e "${YELLOW}⏳ Waiting 10 seconds for CARLA to start...${NC}"
    sleep 10
else
    echo -e "${GREEN}✅ CARLA is already running${NC}"
fi

# Setup environment
cd "$(dirname "$0")"
CARLA_DIR="${CARLA_DIR:-/home/a/Desktop/CARLA_0.9.16}"
CARLA_EGG=$(find "$CARLA_DIR/PythonAPI/carla/dist" -name "carla-*.egg" 2>/dev/null | head -1)
if [ -n "$CARLA_EGG" ]; then
    export PYTHONPATH="$CARLA_EGG:$PYTHONPATH"
fi

# Mode
MODE="${1:-inference}"
echo -e "${BLUE}Mode: ${MODE}${NC}"
echo ""

# Run
echo -e "${GREEN}🚀 Starting system...${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop${NC}"
echo ""

python3 main.py --mode "$MODE"

