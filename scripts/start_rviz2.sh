#!/bin/bash
# Start RViz2 for CARLA visualization

echo "🎨 Starting RViz2..."

# Source ROS2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
else
    echo "❌ ROS2 not found!"
    exit 1
fi

# Start RViz2 with CARLA config
echo "🚀 Launching RViz2..."
echo ""
echo "Add these topics to visualize:"
echo "  - /carla/ego_vehicle/lidar/point_cloud2 (PointCloud2)"
echo "  - /carla/ego_vehicle/camera/rgb/image_raw (Image)"
echo "  - /carla/ego_vehicle/odometry (Odometry)"
echo ""

rviz2 -d config/carla_rviz.rviz 2>/dev/null || rviz2
