#!/bin/bash

# Start Complete Training System with ROS2
# เริ่มระบบทั้งหมด: ROS2 Bridge + Training with Guidelines

set -e

echo "================================================================================"
echo "🚀 Starting Complete Training System with ROS2"
echo "================================================================================"
echo ""

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Check CARLA is running
echo -e "${YELLOW}🔍 Checking CARLA Server...${NC}"
if ! pgrep -f "CarlaUE4" > /dev/null; then
    echo -e "${RED}❌ CARLA Server is not running!${NC}"
    echo "Please start CARLA first:"
    echo "  cd /home/supawich/Desktop/CARLA_0.9.16"
    echo "  ./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2000"
    exit 1
fi
echo -e "${GREEN}✅ CARLA Server is running${NC}"
echo ""

# Activate Python environment
echo -e "${YELLOW}🐍 Activating Python environment...${NC}"
cd /home/supawich/Desktop/carla_sac_ros2_training
source venv/bin/activate
echo -e "${GREEN}✅ Python environment activated${NC}"
echo ""

# Check if ROS2 is available
echo -e "${YELLOW}🤖 Checking ROS2...${NC}"
if command -v ros2 &> /dev/null; then
    echo -e "${GREEN}✅ ROS2 is available${NC}"
    
    # Source ROS2
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        echo -e "${GREEN}✅ Sourced ROS2 Humble${NC}"
    elif [ -f "/opt/ros/foxy/setup.bash" ]; then
        source /opt/ros/foxy/setup.bash
        echo -e "${GREEN}✅ Sourced ROS2 Foxy${NC}"
    fi
    
    # Build ROS2 workspace if needed
    if [ -d "ros2_ws" ]; then
        echo -e "${YELLOW}🔨 Building ROS2 workspace...${NC}"
        cd ros2_ws
        colcon build --symlink-install 2>&1 | grep -E "(Starting|Finished|Summary)" || true
        source install/setup.bash
        cd ..
        echo -e "${GREEN}✅ ROS2 workspace ready${NC}"
    fi
    
    USE_ROS2=true
else
    echo -e "${YELLOW}⚠️  ROS2 not available, using Python-only mode${NC}"
    USE_ROS2=false
fi
echo ""

# Start training
echo "================================================================================"
echo -e "${GREEN}🎓 Starting Training with Guidelines${NC}"
echo "================================================================================"
echo ""
echo "Configuration:"
echo "  - Training Guidelines: ENABLED"
echo "  - Curriculum Learning: ENABLED (3 stages)"
echo "  - MLflow Tracking: ENABLED"
echo "  - ROS2 Integration: $USE_ROS2"
echo ""
echo "Guidelines:"
echo "  - Target Speed: 30 km/h"
echo "  - Lane Keeping: ±0.5m acceptable"
echo "  - Steering Smoothness: max 0.2 change/step"
echo "  - Collision Penalty: -200"
echo ""
echo "Press Ctrl+C to stop training"
echo ""
sleep 2

# Run training
python scripts/train_with_guidelines.py --episodes 1000 --expert

echo ""
echo "================================================================================"
echo -e "${GREEN}✅ Training Complete!${NC}"
echo "================================================================================"
echo ""
echo "View results:"
echo "  - MLflow UI: http://localhost:5000"
echo "  - Tensorboard: http://localhost:6006"
echo ""
