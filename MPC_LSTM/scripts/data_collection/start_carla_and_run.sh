#!/bin/bash
# Start CARLA and run complete auto pipeline

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

cd "$(dirname "$0")"

CARLA_DIR=/home/a/Desktop/CARLA_0.9.16

echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}🚀 Starting CARLA + Complete Auto Pipeline${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo ""

# Check if CARLA is already running
if pgrep -f "CarlaUE4" > /dev/null; then
    echo -e "${GREEN}✅ CARLA is already running${NC}"
else
    echo -e "${YELLOW}Starting CARLA...${NC}"
    cd "$CARLA_DIR"
    ./CarlaUE4.sh > /dev/null 2>&1 &
    CARLA_PID=$!
    echo -e "${GREEN}✅ CARLA started (PID: $CARLA_PID)${NC}"
    
    # Wait for CARLA to be ready
    echo -e "${YELLOW}Waiting for CARLA to be ready...${NC}"
    sleep 10
    
    # Check if CARLA is responding
    for i in {1..30}; do
        if timeout 2 python3 -c "import carla; client = carla.Client('localhost', 2000); client.set_timeout(2.0); client.get_world()" 2>/dev/null; then
            echo -e "${GREEN}✅ CARLA is ready!${NC}"
            break
        fi
        echo -n "."
        sleep 2
    done
    echo ""
fi

cd "$(dirname "$0")"

# Run complete auto pipeline
echo -e "${BLUE}Starting complete auto pipeline...${NC}"
echo ""

./run_complete_auto.sh

echo ""
echo -e "${GREEN}════════════════════════════════════════${NC}"
echo -e "${GREEN}✅ All Done!${NC}"
echo -e "${GREEN}════════════════════════════════════════${NC}"

