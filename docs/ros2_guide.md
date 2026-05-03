# 🤖 ROS2 Integration Guide

**วันที่:** 10 เมษายน 2026  
**สถานะ:** ✅ พร้อมใช้งาน

---

## ✅ ROS2 Features ที่มีอยู่

### 1. **ROS2 CARLA Bridge** 🌉
- เชื่อมต่อ CARLA กับ ROS2
- Publish sensor data เป็น ROS2 topics
- Subscribe control commands จาก ROS2

### 2. **Sensor Publishers** 📡
- **LiDAR:** PointCloud2 messages
- **Camera:** Image messages (RGB)
- **IMU:** IMU messages
- **Odometry:** Odometry messages
- **Velocity:** TwistStamped messages

### 3. **Control Subscriber** 🎮
- Subscribe vehicle control commands
- Apply controls to CARLA vehicle

---

## 🚀 วิธีใช้งาน ROS2

### Prerequisites:
```bash
# ต้องติดตั้ง ROS2 ก่อน (Humble หรือ Foxy)
# Ubuntu 22.04: ROS2 Humble
# Ubuntu 20.04: ROS2 Foxy
```

### Quick Start (3 Terminals):

**Terminal 1: CARLA Server**
```bash
cd /home/supawich/Desktop/CARLA_0.9.16
./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2000
```

**Terminal 2: ROS2 Bridge**
```bash
cd /home/supawich/Desktop/carla_sac_ros2_training
./scripts/start_ros2_bridge.sh
```

**Terminal 3: RViz2 (Visualization)**
```bash
cd /home/supawich/Desktop/carla_sac_ros2_training
./scripts/start_rviz2.sh
```

---

## 📡 ROS2 Topics

### Published Topics:

| Topic | Type | Description | Rate |
|-------|------|-------------|------|
| `/carla/ego_vehicle/lidar/point_cloud2` | `sensor_msgs/PointCloud2` | LiDAR point cloud | 20 Hz |
| `/carla/ego_vehicle/camera/rgb/image_raw` | `sensor_msgs/Image` | RGB camera image | 20 Hz |
| `/carla/ego_vehicle/odometry` | `nav_msgs/Odometry` | Vehicle odometry | 50 Hz |
| `/carla/ego_vehicle/imu` | `sensor_msgs/Imu` | IMU data | 50 Hz |
| `/carla/ego_vehicle/velocity` | `geometry_msgs/TwistStamped` | Vehicle velocity | 50 Hz |

### Subscribed Topics:

| Topic | Type | Description |
|-------|------|-------------|
| `/carla/ego_vehicle/control` | `ackermann_msgs/AckermannDrive` | Vehicle control |

---

## 🎨 RViz2 Visualization

### การตั้งค่า RViz2:

1. **เพิ่ม PointCloud2 Display:**
   - Add → PointCloud2
   - Topic: `/carla/ego_vehicle/lidar/point_cloud2`
   - Fixed Frame: `ego_vehicle`
   - Color: Intensity

2. **เพิ่ม Image Display:**
   - Add → Image
   - Topic: `/carla/ego_vehicle/camera/rgb/image_raw`

3. **เพิ่ม Odometry Display:**
   - Add → Odometry
   - Topic: `/carla/ego_vehicle/odometry`
   - Keep: 100
   - Shaft Length: 1.0

4. **เพิ่ม TF Display:**
   - Add → TF
   - Show Names: true
   - Show Axes: true

---

## 🔧 การใช้งานร่วมกับ Training

### Scenario 1: Training + ROS2 Logging

```python
from src.carla_gym_env import CarlaEnv
from src.ros2_bridge import CarlaRosNode
import rclpy

# Initialize ROS2
rclpy.init()

# Create environment
env = CarlaEnv(config)
obs, _ = env.reset()

# Create ROS2 node
ros_node = CarlaRosNode(env.world, env.vehicle)

# Training loop
for episode in range(1000):
    obs, _ = env.reset()
    done = False
    
    while not done:
        action = policy.get_action(obs)
        obs, reward, done, truncated, info = env.step(action)
        
        # ROS2 node จะ publish sensor data อัตโนมัติ
        rclpy.spin_once(ros_node, timeout_sec=0)

# Cleanup
ros_node.destroy_node()
rclpy.shutdown()
```

### Scenario 2: Remote Control via ROS2

```bash
# Terminal 1: CARLA + ROS2 Bridge
./scripts/start_ros2_bridge.sh

# Terminal 2: Publish control commands
ros2 topic pub /carla/ego_vehicle/control \
  ackermann_msgs/msg/AckermannDrive \
  "{steering_angle: 0.0, speed: 5.0}"
```

---

## 📊 ROS2 + MLflow Integration

### Log ROS2 Topics to MLflow:

```python
from src.mlflow_integration import MLflowTracker
import rclpy
from rclpy.node import Node

class MLflowRosLogger(Node):
    def __init__(self, tracker: MLflowTracker):
        super().__init__('mlflow_logger')
        self.tracker = tracker
        
        # Subscribe to topics
        self.create_subscription(
            Odometry,
            '/carla/ego_vehicle/odometry',
            self.odom_callback,
            10
        )
        
        self.step = 0
    
    def odom_callback(self, msg):
        # Log to MLflow
        self.tracker.log_metric(
            'ros2/speed',
            msg.twist.twist.linear.x,
            step=self.step
        )
        self.step += 1

# Usage
tracker = MLflowTracker()
tracker.start_run()

rclpy.init()
logger = MLflowRosLogger(tracker)
rclpy.spin(logger)
```

---

## 🛠️ Useful ROS2 Commands

### ดู Topics ทั้งหมด:
```bash
ros2 topic list
```

### ดูข้อมูลใน Topic:
```bash
ros2 topic echo /carla/ego_vehicle/odometry
```

### ดู Topic Info:
```bash
ros2 topic info /carla/ego_vehicle/lidar/point_cloud2
```

### ดู Message Type:
```bash
ros2 interface show sensor_msgs/msg/PointCloud2
```

### Record Topics (Bag File):
```bash
ros2 bag record -a  # Record all topics
# หรือ
ros2 bag record /carla/ego_vehicle/lidar/point_cloud2 \
                /carla/ego_vehicle/camera/rgb/image_raw
```

### Playback Bag File:
```bash
ros2 bag play <bag_file>
```

---

## 🎯 Use Cases

### 1. **Data Collection for Training**
- Record sensor data เป็น ROS2 bags
- Replay สำหรับ offline training
- Share data ระหว่าง researchers

### 2. **Multi-Robot Systems**
- ใช้ ROS2 DDS สำหรับ communication
- Coordinate multiple vehicles
- Distributed training

### 3. **Hardware-in-the-Loop Testing**
- Test algorithms กับ real sensors
- Bridge CARLA กับ real robot
- Validate before deployment

### 4. **Visualization & Debugging**
- ใช้ RViz2 ดู sensor data
- Plot topics ด้วย PlotJuggler
- Debug algorithms แบบ real-time

---

## 📦 ROS2 Packages ที่ใช้

```xml
<!-- package.xml -->
<package format="3">
  <name>carla_sac_ros2</name>
  <version>1.0.0</version>
  <description>CARLA SAC Training with ROS2</description>
  
  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>ackermann_msgs</depend>
  <depend>tf2_ros</depend>
</package>
```

---

## 🔍 Troubleshooting

### ปัญหา: ROS2 not found
```bash
# ติดตั้ง ROS2 Humble (Ubuntu 22.04)
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash

# หรือ ROS2 Foxy (Ubuntu 20.04)
sudo apt install ros-foxy-desktop
source /opt/ros/foxy/setup.bash
```

### ปัญหา: Topics ไม่ปรากฏ
```bash
# ตรวจสอบว่า ROS2 bridge กำลังรัน
ros2 node list

# ตรวจสอบ topics
ros2 topic list

# ตรวจสอบ network
ros2 doctor
```

### ปัญหา: RViz2 ไม่แสดงผล
```bash
# ตรวจสอบ Fixed Frame
# ใน RViz2 Global Options → Fixed Frame → เปลี่ยนเป็น "ego_vehicle"

# ตรวจสอบว่ามี TF transforms
ros2 run tf2_ros tf2_echo map ego_vehicle
```

---

## 🎓 Advanced: Custom ROS2 Nodes

### สร้าง Custom Subscriber:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class CustomLidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/carla/ego_vehicle/lidar/point_cloud2',
            self.lidar_callback,
            10
        )
    
    def lidar_callback(self, msg):
        # Process LiDAR data
        print(f"Received {msg.width * msg.height} points")
        
        # Your processing here...

def main():
    rclpy.init()
    node = CustomLidarProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 📝 สรุป

### ✅ ROS2 Features:
- ✅ CARLA ROS2 Bridge
- ✅ Sensor Publishers (LiDAR, Camera, IMU, Odometry)
- ✅ Control Subscriber
- ✅ RViz2 Visualization
- ✅ Topic Recording/Playback
- ✅ MLflow Integration

### 🚀 Quick Commands:
```bash
# เริ่ม ROS2 Bridge
./scripts/start_ros2_bridge.sh

# เริ่ม RViz2
./scripts/start_rviz2.sh

# ดู Topics
ros2 topic list

# Record Data
ros2 bag record -a
```

### 🌐 Integration:
- ✅ ใช้ร่วมกับ Training Pipeline
- ✅ Log ไป MLflow
- ✅ Visualize ด้วย RViz2
- ✅ Record/Replay สำหรับ analysis

**ROS2 พร้อมใช้งานแล้ว!** 🤖
