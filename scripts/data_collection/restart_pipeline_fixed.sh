#!/bin/bash
# Restart pipeline with fixed autopilot

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CARLA_DIR="${CARLA_DIR:-/home/a/Desktop/CARLA_0.9.16}"

echo -e "${BLUE}🔄 Restarting Pipeline with Fixes${NC}"
echo "===================================="
echo ""

# Setup
export HSA_OVERRIDE_GFX_VERSION=11.0.0
CARLA_EGG=$(find "$CARLA_DIR/PythonAPI/carla/dist" -name "carla-*.egg" 2>/dev/null | head -1)
if [ -n "$CARLA_EGG" ]; then
    export PYTHONPATH="$CARLA_EGG:$PYTHONPATH"
fi

cd "$PROJECT_DIR"

# Check CARLA
echo -e "${YELLOW}Checking CARLA...${NC}"
if ! python3 -c "import carla; client = carla.Client('localhost', 2000); client.set_timeout(5.0); client.get_world()" 2>/dev/null; then
    echo -e "${RED}❌ CARLA not accessible${NC}"
    exit 1
fi
echo -e "${GREEN}✅ CARLA ready${NC}"
echo ""

# Start fresh collection with fixed autopilot
FRAMES=${1:-20000}
EPOCHS=${2:-30}

echo -e "${BLUE}Starting fresh data collection (${FRAMES} frames)...${NC}"
echo -e "${YELLOW}Note: Make sure CARLA window is visible and vehicle can move${NC}"
echo ""

python3 training/collect_autopilot_data.py --frames "$FRAMES"

DATA_DIR=$(ls -td data/autopilot_* | head -1)
echo -e "${GREEN}✅ Data collection complete: $DATA_DIR${NC}"

# Check data quality
echo ""
echo -e "${BLUE}Checking data quality...${NC}"
python3 << EOF
import pandas as pd
import sys

df = pd.read_csv('$DATA_DIR/data.csv')
distance = ((df['x'].diff()**2 + df['y'].diff()**2)**0.5).sum()
moving = (df['velocity'] > 1.0).sum()

print(f"Total frames: {len(df)}")
print(f"Moving frames (v>1.0): {moving}")
print(f"Distance traveled: {distance:.2f} m")

if distance < 10:
    print("⚠️  WARNING: Vehicle barely moved!")
    print("   Data may not be suitable for training")
    sys.exit(1)
else:
    print("✅ Data quality OK")
EOF

if [ $? -ne 0 ]; then
    echo -e "${RED}❌ Data quality check failed${NC}"
    exit 1
fi

echo ""
echo -e "${BLUE}Continuing with preprocessing...${NC}"

# Continue pipeline
./fix_and_continue_pipeline.sh "$EPOCHS"

