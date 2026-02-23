#!/bin/bash
# Run CARLA Autopilot with Ultra-Fast-Lane-Detection-v2 Demo

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

cd "$(dirname "$0")/../.."

echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}CARLA Autopilot + Lane Detection Demo${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo ""

# Setup environment
export HSA_OVERRIDE_GFX_VERSION=11.0.0
export AMD_SERIALIZE_KERNEL=3
export HIP_FORCE_DEV_KERNELS=1

CARLA_DIR="${CARLA_DIR:-/home/a/Desktop/CARLA_0.9.16}"
CARLA_EGG=$(find "$CARLA_DIR/PythonAPI/carla/dist" -name "carla-*.egg" 2>/dev/null | head -1)
if [ -n "$CARLA_EGG" ]; then
    export PYTHONPATH="$CARLA_EGG:$PYTHONPATH"
fi

# Check if CARLA is running
echo -e "${YELLOW}Checking CARLA...${NC}"
if ! timeout 5 python3 -c "import carla; c=carla.Client('localhost',2000); c.set_timeout(3); c.get_world()" 2>/dev/null; then
    echo -e "${YELLOW}⚠️  CARLA not running, starting...${NC}"
    cd "$CARLA_DIR"
    ./CarlaUE4.sh -quality-level=Low -prefernoloadscreen > /tmp/carla.log 2>&1 &
    echo -e "${YELLOW}⏳ Waiting 40 seconds for CARLA to load...${NC}"
    sleep 40
    cd "$(dirname "$0")/../.."
    
    # Check again
    if ! timeout 10 python3 -c "import carla; c=carla.Client('localhost',2000); c.set_timeout(5); c.get_world()" 2>/dev/null; then
        echo -e "${RED}❌ CARLA failed to start${NC}"
        exit 1
    fi
fi

echo -e "${GREEN}✅ CARLA is ready${NC}"
echo ""

# Run demo
echo -e "${BLUE}Starting demo...${NC}"
echo -e "${YELLOW}Press 'q' in the window to quit${NC}"
echo ""

python3 scripts/demo/run_autopilot_lane_detection.py "$@"

echo ""
echo -e "${GREEN}✅ Demo complete${NC}"

