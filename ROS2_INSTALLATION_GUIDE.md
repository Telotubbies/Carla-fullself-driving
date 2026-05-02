# 🚀 ROS2 Installation & Usage Guide

**วันที่:** 10 เมษายน 2026  
**สถานะ:** ✅ พร้อมติดตั้งและใช้งาน

---

## 📋 ข้อกำหนด

### System Requirements:
- Ubuntu 22.04 (ROS2 Humble) หรือ Ubuntu 20.04 (ROS2 Foxy)
- Python 3.8+
- CARLA 0.9.16
- 8GB RAM ขึ้นไป

---

## 🔧 Installation

### Step 1: ติดตั้ง ROS2

#### Ubuntu 22.04 (ROS2 Humble):
```bash
# Add ROS2 repository
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Install additional packages
sudo apt install ros-humble-cv-bridge
sudo apt install ros-humble-vision-opencv
sudo apt install python3-colcon-common-extensions

# Source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### Ubuntu 20.04 (ROS2 Foxy):
```bash
# Add ROS2 repository
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe

# Install ROS2 Foxy
sudo apt update
sudo apt install ros-foxy-desktop

# Install additional packages
sudo apt install ros-foxy-cv-bridge
sudo apt install ros-foxy-vision-opencv
sudo apt install python3-colcon-common-extensions

# Source ROS2
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Build ROS2 Workspace

```bash
cd /home/supawich/Desktop/carla_sac_ros2_training/ros2_ws

# Source ROS2
source /opt/ros/humble/setup.bash  # หรือ foxy

# Build packages
colcon build --symlink-install

# Source workspace
source install/setup.bash

# Add to bashrc (optional)
echo "source /home/supawich/Desktop/carla_sac_ros2_training/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Step 3: Verify Installation

```bash
# Check ROS2 version
ros2 --version

# List packages
ros2 pkg list | grep carla

# Should see:
# carla_sac_bridge
```

---

## 🚀 Usage

### Quick Start (4 Terminals):

#### Terminal 1: CARLA Server
```bash
cd /home/supawich/Desktop/CARLA_0.9.16
./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2000
```

#### Terminal 2: Source ROS2 & Launch System
```bash
cd /home/supawich/Desktop/carla_sac_ros2_training/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch everything
ros2 launch carla_sac_bridge carla_system.launch.py
```

#### Terminal 3: Monitor Topics (Optional)
```bash
source /opt/ros/humble/setup.bash

# List topics
ros2 topic list

# Echo specific topic
ros2 topic echo /carla/ego_vehicle/odometry

# Monitor metrics
ros2 topic echo /carla/training/metrics
```

#### Terminal 4: MLflow UI (Optional)
```bash
cd /home/supawich/Desktop/carla_sac_ros2_training
./scripts/start_mlflow_ui.sh
```

---

## 📡 ROS2 Topics

### Published Topics:

| Topic | Type | Description | Rate |
|-------|------|-------------|------|
| `/carla/ego_vehicle/camera/rgb/image_raw` | `sensor_msgs/Image` | Camera RGB | 30 Hz |
| `/carla/ego_vehicle/lidar/point_cloud2` | `sensor_msgs/PointCloud2` | LiDAR | 20 Hz |
| `/carla/ego_vehicle/odometry` | `nav_msgs/Odometry` | Vehicle odometry | 50 Hz |
| `/carla/ego_vehicle/velocity` | `geometry_msgs/TwistStamped` | Velocity | 50 Hz |
| `/carla/training/metrics` | `std_msgs/Float32MultiArray` | Training metrics | 20 Hz |
| `/carla/training/status` | `std_msgs/String` | Training status | 1 Hz |

### Subscribed Topics:

| Topic | Type | Description |
|-------|------|-------------|
| `/carla/ego_vehicle/control` | `geometry_msgs/Twist` | Vehicle control |

---

## ⚙️ Configuration

### Launch Arguments:

```bash
# Custom CARLA host/port
ros2 launch carla_sac_bridge carla_system.launch.py \
  carla_host:=192.168.1.100 \
  carla_port:=2000

# Disable guidelines
ros2 launch carla_sac_bridge carla_system.launch.py \
  use_guidelines:=false

# Disable curriculum
ros2 launch carla_sac_bridge carla_system.launch.py \
  curriculum_enabled:=false
```

### Edit Configuration:

```bash
# Edit config file
nano ros2_ws/src/carla_sac_bridge/config/carla_config.yaml

# Rebuild
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 🎮 Running Individual Nodes

### Run Bridge Only:
```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 run carla_sac_bridge carla_bridge
```

### Run Camera Monitor Only:
```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 run carla_sac_bridge camera_monitor
```

---

## 🔍 Debugging

### Check Node Status:
```bash
# List running nodes
ros2 node list

# Node info
ros2 node info /carla_bridge

# Topic info
ros2 topic info /carla/ego_vehicle/camera/rgb/image_raw
```

### Check Topic Data:
```bash
# Echo topic
ros2 topic echo /carla/training/metrics

# Topic frequency
ros2 topic hz /carla/ego_vehicle/camera/rgb/image_raw

# Topic bandwidth
ros2 topic bw /carla/ego_vehicle/camera/rgb/image_raw
```

### View Logs:
```bash
# View node logs
ros2 run rqt_console rqt_console

# Or use terminal
ros2 topic echo /rosout
```

---

## 📊 Features

### ✅ Training Guidelines Integration
- Speed limits (30 km/h target)
- Steering smoothness (max 0.2 change/step)
- Lane keeping (±0.5m acceptable)
- Collision penalty (-200)
- Automatic reward calculation

### ✅ Curriculum Learning
- Stage 1: Basic Control (0-500 eps)
- Stage 2: Navigation (500-1500 eps)
- Stage 3: Complex (1500+ eps)
- Auto stage transition
- Progress tracking

### ✅ Real-time Monitoring
- Camera view with overlay
- Speed, steering, throttle
- Lane deviation
- Reward tracking
- FPS counter
- Status updates

### ✅ ROS2 Best Practices
- Proper package structure
- QoS profiles
- Launch files
- Configuration files
- Parameter server
- Topic namespacing

---

## 🐛 Troubleshooting

### Problem: ROS2 not found
```bash
# Check installation
ros2 --version

# Re-source
source /opt/ros/humble/setup.bash
```

### Problem: Package not found
```bash
# Rebuild workspace
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Problem: CARLA connection failed
```bash
# Check CARLA is running
ps aux | grep Carla

# Check port
netstat -tulpn | grep 2000

# Test connection
python3 -c "import carla; client = carla.Client('localhost', 2000); print(client.get_server_version())"
```

### Problem: Camera not showing
```bash
# Check topic
ros2 topic list | grep camera

# Check data
ros2 topic echo /carla/ego_vehicle/camera/rgb/image_raw --once

# Check cv_bridge
python3 -c "from cv_bridge import CvBridge; print('OK')"
```

### Problem: Build errors
```bash
# Clean build
cd ros2_ws
rm -rf build install log
colcon build --symlink-install

# Check dependencies
rosdep install --from-paths src --ignore-src -r -y
```

---

## 📝 Example Workflow

### Complete Training Session:

```bash
# Terminal 1: Start CARLA
cd /home/supawich/Desktop/CARLA_0.9.16
./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2000

# Terminal 2: Start ROS2 System
cd /home/supawich/Desktop/carla_sac_ros2_training/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch carla_sac_bridge carla_system.launch.py

# Terminal 3: Monitor Metrics
source /opt/ros/humble/setup.bash
ros2 topic echo /carla/training/metrics

# Terminal 4: MLflow UI
cd /home/supawich/Desktop/carla_sac_ros2_training
./scripts/start_mlflow_ui.sh
```

**จะได้:**
- ✅ CARLA running
- ✅ ROS2 bridge publishing data
- ✅ Camera monitor showing view
- ✅ Guidelines applied
- ✅ Curriculum learning active
- ✅ MLflow tracking

---

## 🎯 Next Steps

### 1. Test System:
```bash
ros2 launch carla_sac_bridge carla_system.launch.py
```

### 2. Monitor Topics:
```bash
ros2 topic list
ros2 topic echo /carla/training/status
```

### 3. View Camera:
- Camera window จะเปิดอัตโนมัติ
- แสดง metrics overlay
- กด 'q' เพื่อออก

### 4. Check Guidelines:
- ดู `/carla/training/metrics` สำหรับ reward components
- ดู `/carla/training/status` สำหรับ curriculum progress

---

## 📚 Documentation

- **ROS2 Humble:** https://docs.ros.org/en/humble/
- **ROS2 Foxy:** https://docs.ros.org/en/foxy/
- **CARLA:** https://carla.readthedocs.io/
- **Training Guidelines:** `config/training_guidelines.yaml`
- **Hybrid Guide:** `ROS2_HYBRID_GUIDE.md`

---

## 🎉 สรุป

**ระบบ ROS2 พร้อมใช้งาน!**

✅ ROS2 package structure ตาม best practices  
✅ Training guidelines integration  
✅ Curriculum learning  
✅ Real-time camera monitoring  
✅ Metrics tracking  
✅ Configuration files  
✅ Launch files  

**เริ่มใช้งาน:**
```bash
ros2 launch carla_sac_bridge carla_system.launch.py
```

**มีความเสถียร มี guidelines ชัดเจน และใช้ ROS2 ตาม documentation!** 🚀
