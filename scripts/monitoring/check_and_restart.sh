#!/bin/bash
# Check status and restart if needed

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

cd "$(dirname "$0")"

echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}🔍 Checking System Status${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo ""

# Check training
TRAINING_RUNNING=$(pgrep -f "train_lstm.py" > /dev/null && echo "yes" || echo "no")
if [ "$TRAINING_RUNNING" = "yes" ]; then
    echo -e "${YELLOW}⚠️  Training is still running${NC}"
    echo -e "${YELLOW}   Do you want to wait for it to complete? (y/n)${NC}"
    read -t 5 answer || answer="n"
    if [ "$answer" = "y" ]; then
        echo -e "${YELLOW}Waiting for training to complete...${NC}"
        while pgrep -f "train_lstm.py" > /dev/null; do
            echo -n "."
            sleep 5
        done
        echo ""
        echo -e "${GREEN}✅ Training completed!${NC}"
    else
        echo -e "${YELLOW}Keeping training running...${NC}"
    fi
else
    echo -e "${GREEN}✅ No training process running${NC}"
fi

# Check CARLA
CARLA_RUNNING=$(pgrep -f "CarlaUE4" > /dev/null && echo "yes" || echo "no")
if [ "$CARLA_RUNNING" = "no" ]; then
    echo -e "${RED}❌ CARLA is not running${NC}"
    echo -e "${YELLOW}   Will start CARLA automatically...${NC}"
    RESTART_CARLA="yes"
else
    echo -e "${GREEN}✅ CARLA is running${NC}"
    RESTART_CARLA="no"
fi

# Check main/inference
MAIN_RUNNING=$(pgrep -f "main.py.*inference" > /dev/null && echo "yes" || echo "no")
if [ "$MAIN_RUNNING" = "yes" ]; then
    echo -e "${YELLOW}⚠️  Inference is already running${NC}"
    echo -e "${YELLOW}   Do you want to stop it and restart? (y/n)${NC}"
    read -t 5 answer || answer="n"
    if [ "$answer" = "y" ]; then
        echo -e "${YELLOW}Stopping inference...${NC}"
        pkill -f "main.py.*inference" || true
        sleep 2
        RESTART_INFERENCE="yes"
    else
        echo -e "${YELLOW}Keeping inference running...${NC}"
        RESTART_INFERENCE="no"
    fi
else
    echo -e "${GREEN}✅ No inference process running${NC}"
    RESTART_INFERENCE="yes"
fi

echo ""
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}🚀 Restarting System${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo ""

# Restart CARLA if needed
if [ "$RESTART_CARLA" = "yes" ]; then
    echo -e "${YELLOW}Starting CARLA...${NC}"
    CARLA_DIR=/home/a/Desktop/CARLA_0.9.16
    cd "$CARLA_DIR"
    ./CarlaUE4.sh > /dev/null 2>&1 &
    CARLA_PID=$!
    echo -e "${GREEN}✅ CARLA started (PID: $CARLA_PID)${NC}"
    
    # Wait for CARLA
    echo -e "${YELLOW}Waiting for CARLA to be ready...${NC}"
    sleep 10
    
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
if [ "$RESTART_INFERENCE" = "yes" ]; then
    echo -e "${BLUE}Starting complete auto pipeline...${NC}"
    echo ""
    ./run_complete_auto.sh
else
    echo -e "${GREEN}✅ System is already running!${NC}"
fi

