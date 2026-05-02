#!/bin/bash
# Start ROS2 CARLA Bridge

echo "🤖 Starting ROS2 CARLA Bridge..."
echo ""

# Source ROS2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ ROS2 Humble sourced"
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
    echo "✅ ROS2 Foxy sourced"
else
    echo "❌ ROS2 not found! Please install ROS2 first."
    exit 1
fi

# Source workspace
cd /home/supawich/Desktop/carla_sac_ros2_training
source venv/bin/activate

echo ""
echo "🚀 Starting CARLA ROS2 Bridge Node..."
echo "Topics will be published to:"
echo "  - /carla/ego_vehicle/lidar/point_cloud2"
echo "  - /carla/ego_vehicle/camera/rgb/image_raw"
echo "  - /carla/ego_vehicle/odometry"
echo "  - /carla/ego_vehicle/imu"
echo ""

python scripts/run_ros2_bridge.py
