#!/bin/bash
# Fix stuck vehicle and restart everything

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

cd "$(dirname "$0")"

echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}🔧 Fixing and Restarting System${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo ""

# Stop all processes
echo -e "${YELLOW}Stopping all processes...${NC}"
pkill -f "main.py" || true
pkill -f "train_lstm.py" || true
sleep 2
echo -e "${GREEN}✅ Stopped${NC}"
echo ""

# Restart CARLA
echo -e "${YELLOW}Restarting CARLA...${NC}"
pkill -f "CarlaUE4" || true
sleep 3

CARLA_DIR=/home/a/Desktop/CARLA_0.9.16
cd "$CARLA_DIR"
./CarlaUE4.sh > /dev/null 2>&1 &
CARLA_PID=$!
echo -e "${GREEN}✅ CARLA started (PID: $CARLA_PID)${NC}"

# Wait for CARLA
echo -e "${YELLOW}Waiting for CARLA to be ready...${NC}"
sleep 15

# Check CARLA
for i in {1..30}; do
    if timeout 2 python3 -c "import carla; client = carla.Client('localhost', 2000); client.set_timeout(2.0); client.get_world()" 2>/dev/null; then
        echo -e "${GREEN}✅ CARLA is ready!${NC}"
        break
    fi
    echo -n "."
    sleep 2
done
echo ""

cd "$(dirname "$0")"

# Run inference
echo -e "${BLUE}Starting inference...${NC}"
export HSA_OVERRIDE_GFX_VERSION=11.0.0
export CARLA_DIR=/home/a/Desktop/CARLA_0.9.16
CARLA_EGG=$(find "$CARLA_DIR/PythonAPI/carla/dist" -name "carla-*.egg" 2>/dev/null | head -1)
export PYTHONPATH="$CARLA_EGG:$PYTHONPATH"

python3 main.py --mode inference

