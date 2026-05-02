# 🚀 Quick Start - ROS2 Camera Monitor

**วันที่:** 10 เมษายน 2026  
**สถานะ:** ✅ พร้อมใช้งาน (ไม่ต้องติดตั้ง ROS2 ก็ได้)

---

## 🎯 สองวิธีในการใช้งาน

### วิธีที่ 1: ใช้ Python Visualization (ไม่ต้องมี ROS2) ✅ แนะนำ
- ใช้ `visualize_advanced.py` ที่มีอยู่แล้ว
- ดูกล้อง + LiDAR + Metrics ครบ
- ไม่ต้องติดตั้ง ROS2

### วิธีที่ 2: ใช้ ROS2 (ถ้าต้องการ performance สูง)
- C++ สำหรับ processing
- Python สำหรับดูกล้อง
- ต้องติดตั้ง ROS2

---

## 🎨 วิธีที่ 1: Python Visualization (ง่ายที่สุด)

### เริ่มใช้งานเลย (3 Terminals):

**Terminal 1: CARLA Server**
```bash
cd /home/supawich/Desktop/CARLA_0.9.16
./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2000
```

**Terminal 2: MLflow UI**
```bash
cd /home/supawich/Desktop/carla_sac_ros2_training
./scripts/start_mlflow_ui.sh
```

**Terminal 3: Advanced Visualization**
```bash
cd /home/supawich/Desktop/carla_sac_ros2_training
source venv/bin/activate
python visualize_advanced.py
```

### จะได้อะไร:
- ✅ มุมมองกล้อง (640x480)
- ✅ LiDAR BEV
- ✅ Metrics (speed, steering, rewards)
- ✅ Curriculum progress
- ✅ Performance stats
- ✅ Episode info

**ไม่ต้องติดตั้งอะไรเพิ่ม - พร้อมใช้เลย!**

---

## 🤖 วิธีที่ 2: ROS2 Camera Monitor (ถ้ามี ROS2)

### Prerequisites:
```bash
# ต้องติดตั้ง ROS2 ก่อน
# Ubuntu 22.04: ROS2 Humble
sudo apt install ros-humble-desktop
sudo apt install ros-humble-cv-bridge

# Ubuntu 20.04: ROS2 Foxy
sudo apt install ros-foxy-desktop
sudo apt install ros-foxy-cv-bridge
```

### วิธีใช้ (4 Terminals):

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

**Terminal 3: Camera Monitor**
```bash
cd /home/supawich/Desktop/carla_sac_ros2_training
source venv/bin/activate

# Source ROS2
source /opt/ros/humble/setup.bash  # หรือ foxy

# Run monitor
python scripts/ros2_camera_monitor.py
```

**Terminal 4: Check Topics**
```bash
# ดู topics ที่มี
ros2 topic list

# ดูข้อมูลใน topic
ros2 topic echo /carla/ego_vehicle/odometry
```

### จะได้อะไร:
- ✅ มุมมองกล้องแบบ real-time
- ✅ Speed, Position
- ✅ FPS counter
- ✅ Heartbeat monitoring
- ✅ กด 'q' เพื่อออก

---

## 📊 เปรียบเทียบ

### Python Visualization (visualize_advanced.py):
```
✅ ไม่ต้องติดตั้ง ROS2
✅ ครบทุกอย่าง (Camera + LiDAR + Metrics)
✅ Curriculum progress
✅ CARLA Spectator mode
⚠️ Performance: ปานกลาง
```

### ROS2 Camera Monitor:
```
✅ Real-time monitoring
✅ Lightweight (แค่ดูกล้อง)
✅ Heartbeat check
✅ ใช้ร่วมกับ C++ node ได้
⚠️ ต้องติดตั้ง ROS2
⚠️ แสดงแค่กล้อง (ไม่มี LiDAR)
```

---

## 🎯 แนะนำ

### สำหรับการใช้งานทั่วไป:
**ใช้ `visualize_advanced.py`**
- ครบทุกอย่าง
- ไม่ต้องติดตั้งเพิ่ม
- เห็นทุก metrics

### สำหรับ Development ที่ต้องการ Performance:
**ใช้ ROS2 Hybrid**
- C++ สำหรับ processing (เร็ว 10-100x)
- Python สำหรับดูกล้อง
- ดู `ROS2_HYBRID_GUIDE.md`

---

## 🔧 Troubleshooting

### ปัญหา: ไม่เห็นกล้อง
```bash
# ตรวจสอบว่า CARLA กำลังรัน
ps aux | grep Carla

# ตรวจสอบว่า environment ถูกต้อง
python -c "import carla; print(carla.__version__)"
```

### ปัญหา: ROS2 ไม่ทำงาน
```bash
# ตรวจสอบว่า ROS2 ติดตั้งแล้ว
ros2 --version

# ตรวจสอบ topics
ros2 topic list

# ตรวจสอบ nodes
ros2 node list
```

### ปัญหา: Performance ช้า
```bash
# ใช้ RenderOffScreen
./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2000

# ลด resolution
# แก้ไขใน config: camera_width, camera_height
```

---

## 📝 สรุป

### ✅ วิธีที่แนะนำ (ง่ายที่สุด):
```bash
# Terminal 1: CARLA
./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2000

# Terminal 2: Visualization
python visualize_advanced.py
```

**เห็นทุกอย่าง: Camera + LiDAR + Metrics + Curriculum**

### 🚀 ถ้าต้องการ Performance สูง:
1. ติดตั้ง ROS2
2. เขียน C++ node สำหรับ processing
3. ใช้ Python แค่ดูกล้อง
4. ดู `ROS2_HYBRID_GUIDE.md`

---

## 🎉 เริ่มใช้งานเลย!

```bash
cd /home/supawich/Desktop/carla_sac_ros2_training
source venv/bin/activate
python visualize_advanced.py
```

**เห็นกล้อง + ทุก metrics + เช็คว่ายังทำงานอยู่!** 🎥✅
