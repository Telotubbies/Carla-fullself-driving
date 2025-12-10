# 🚗 Vehicle Capabilities - รถเราทำอะไรได้บ้างและใช้ Stack อะไรบ้าง

## 🎯 สรุป: รถเราทำอะไรได้บ้าง

### ✅ ฟีเจอร์หลัก

1. **ขับเอง** - Autonomous driving
2. **หลบหลีก** - Obstacle avoidance
3. **หยุดเอง** - Emergency stop
4. **วิ่งตามถนน** - Road following
5. **ตรวจจับ objects** - Object detection
6. **เข้าใจ scene** - Semantic segmentation
7. **วัดระยะ** - Depth estimation
8. **สร้าง map** - HD map building
9. **รู้ตำแหน่ง** - Localization
10. **วางแผนเส้นทาง** - Path planning
11. **ควบคุม** - Vehicle control

---

## 📡 Sensor Stack - ใช้ Sensors อะไรบ้าง

### 1. **RGB Cameras (3 ตัว)**

**ตำแหน่ง:**
- Front camera (หน้า)
- Left camera (ซ้าย)
- Right camera (ขวา)

**Specs:**
- Resolution: 1280x720
- FOV: 110°
- Format: JPEG (quality 90) หรือ PNG

**ใช้ทำอะไร:**
- Object detection (YOLOv11)
- Semantic segmentation (SegFormer)
- Depth estimation (MiDaS)
- BEV feature extraction

**Stack:**
- Hardware: CARLA RGB Camera
- Processing: PIL, OpenCV
- Models: YOLOv11, SegFormer, MiDaS

---

### 2. **LiDAR**

**Specs:**
- Channels: 32
- Range: 100m
- Points per frame: ~25,000
- Format: NPY (x, y, z, intensity)

**ใช้ทำอะไร:**
- HD map building
- Obstacle detection
- Localization (ICP)
- BEV conversion
- Occupancy grid

**Stack:**
- Hardware: CARLA LiDAR
- Processing: Open3D, NumPy
- Algorithms: RANSAC, ICP, BEV rasterization

---

### 3. **Depth Camera**

**Specs:**
- Resolution: 1280x720
- Format: JPEG (logarithmic depth)

**ใช้ทำอะไร:**
- Depth map generation
- 3D scene understanding
- Distance measurement

**Stack:**
- Hardware: CARLA Depth Camera
- Processing: PIL, NumPy

---

### 4. **Semantic Segmentation Camera**

**Specs:**
- Resolution: 1280x720
- Format: JPEG (CityScapes palette)

**ใช้ทำอะไร:**
- Scene understanding
- Road/lane detection
- Object classification

**Stack:**
- Hardware: CARLA Semantic Camera
- Processing: PIL, NumPy
- Models: SegFormer (optional)

---

### 5. **IMU (Inertial Measurement Unit)**

**Data:**
- Accelerometer (x, y, z)
- Gyroscope (x, y, z)
- Compass

**ใช้ทำอะไร:**
- Motion estimation
- Orientation tracking
- GPS fusion

**Stack:**
- Hardware: CARLA IMU
- Processing: NumPy, JSON

---

### 6. **GPS**

**Data:**
- Latitude, Longitude
- Altitude

**ใช้ทำอะไร:**
- Global positioning
- Localization fusion
- Path planning

**Stack:**
- Hardware: CARLA GPS
- Processing: NumPy, JSON

---

## 🧠 Perception Stack - ตรวจจับและเข้าใจ

### 1. **Object Detection**

**Model:** YOLOv11 nano
- **Input**: RGB image (1280x720)
- **Output**: Bounding boxes, classes, confidence
- **Classes**: car, bus, truck, pedestrian, etc.
- **Speed**: ~30-50 FPS (CPU)

**Stack:**
- Framework: PyTorch
- Model: YOLOv11 nano (5.4 MB)
- Library: Ultralytics

---

### 2. **Semantic Segmentation**

**Model:** SegFormer
- **Input**: RGB image (1280x720)
- **Output**: Pixel-wise segmentation (19 classes)
- **Classes**: road, vehicle, pedestrian, building, etc.

**Stack:**
- Framework: PyTorch
- Model: SegFormer-B0 (CityScapes)
- Library: Transformers (HuggingFace)

---

### 3. **Depth Estimation**

**Model:** MiDaS
- **Input**: RGB image (1280x720)
- **Output**: Depth map (metric depth)
- **Range**: 0-100m

**Stack:**
- Framework: PyTorch
- Model: MiDaS DPT-Large (1.28 GB)
- Library: timm, torch

---

### 4. **LiDAR Processing**

**Algorithms:**
- Ground removal (RANSAC)
- BEV rasterization
- Occupancy grid
- ICP localization

**Stack:**
- Library: Open3D, scikit-learn
- Algorithms: RANSAC, ICP, NumPy

---

## 🗺️ HD Map Stack

### **HD Map Building**

**Input:** LiDAR point clouds
**Output:** HD maps
- Height map
- Intensity map
- Drivable area map
- Lane markings

**Stack:**
- Processing: Open3D, NumPy
- Algorithms: RANSAC, BEV rasterization
- Visualization: Matplotlib

---

## 🔄 BEV & Sensor Fusion Stack

### **BEV Fusion**

**Input:**
- RGB features
- LiDAR points

**Output:**
- BEV feature map
- Occupancy grid

**Stack:**
- Framework: PyTorch
- Processing: NumPy, OpenCV
- Fusion: Concat/Add/Multiply

---

## 📍 Localization Stack

### **Localization**

**Methods:**
- LiDAR ICP (Iterative Closest Point)
- GPS fusion
- HD map alignment

**Stack:**
- Library: Open3D, scipy
- Algorithms: ICP, KD-tree
- Processing: NumPy

---

## 🧭 Planning & Control Stack

### **Path Planning**

**Methods:**
- A* algorithm
- Waypoint extraction
- Global route planning

**Stack:**
- Algorithms: A*, Graph search
- Processing: NumPy, scipy

---

### **Vehicle Control**

**Methods:**
- Pure Pursuit Controller
- PID Controller (optional)
- Behavior Agent (CARLA)

**Stack:**
- Control: Pure Pursuit, PID
- CARLA: VehicleControl API
- Processing: NumPy

---

## 🚀 Complete Stack Summary

### **Hardware Layer**
- CARLA Simulator
- Tesla Model 3 vehicle
- 6 sensors (RGB x3, LiDAR, Depth, Semantic, IMU, GPS)

### **Data Collection Layer**
- Python 3.12
- CARLA Python API
- PIL, NumPy
- JPEG compression

### **Perception Layer**
- PyTorch 2.9.1
- YOLOv11 nano (object detection)
- SegFormer (segmentation)
- MiDaS (depth)

### **Processing Layer**
- Open3D (LiDAR)
- OpenCV (image processing)
- scikit-learn (RANSAC)
- scipy (optimization)

### **Planning & Control Layer**
- A* path planning
- Pure Pursuit controller
- Behavior Agent (CARLA)

---

## 🎯 สรุป: รถทำอะไรได้บ้าง

### ✅ Perception (การรับรู้)
1. **ตรวจจับ objects** - YOLOv11 (car, bus, truck, pedestrian)
2. **เข้าใจ scene** - SegFormer (road, lane, building)
3. **วัดระยะ** - MiDaS (depth map)
4. **ตรวจจับ obstacles** - LiDAR (point cloud)

### ✅ Mapping (การสร้างแผนที่)
5. **สร้าง HD map** - LiDAR accumulation
6. **BEV map** - Bird's eye view
7. **Occupancy grid** - Drivable areas

### ✅ Localization (การระบุตำแหน่ง)
8. **รู้ตำแหน่ง** - LiDAR ICP + GPS
9. **Map alignment** - HD map matching

### ✅ Planning (การวางแผน)
10. **วางแผนเส้นทาง** - A* path planning
11. **Waypoint extraction** - HD map waypoints

### ✅ Control (การควบคุม)
12. **ควบคุมรถ** - Pure Pursuit / PID
13. **หลบหลีก** - Behavior Agent
14. **หยุดเอง** - Emergency stop

---

## 📊 Technology Stack

| Layer | Technology |
|-------|------------|
| **Simulator** | CARLA 0.9.16 |
| **Language** | Python 3.12 |
| **Deep Learning** | PyTorch 2.9.1 |
| **Object Detection** | YOLOv11 nano |
| **Segmentation** | SegFormer |
| **Depth** | MiDaS |
| **LiDAR** | Open3D |
| **Image Processing** | OpenCV, PIL |
| **Math/ML** | NumPy, scikit-learn, scipy |
| **Control** | Pure Pursuit, PID |

---

## 🎯 สรุป

**รถเราทำได้:**
- ✅ ตรวจจับ objects, หลบหลีก, หยุดเอง
- ✅ สร้าง map, รู้ตำแหน่ง, วางแผนเส้นทาง
- ✅ ควบคุมรถ, วิ่งตามถนน

**Stack ที่ใช้:**
- ✅ CARLA + Python
- ✅ PyTorch + YOLOv11 + SegFormer + MiDaS
- ✅ Open3D + OpenCV + NumPy
- ✅ Pure Pursuit + A*

**สรุป**: รถเราทำได้ครบทุกอย่างสำหรับ autonomous driving!

