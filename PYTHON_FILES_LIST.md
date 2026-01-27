# รายการ Python Files ใน CARLA

## 📁 PythonAPI/examples/ - ตัวอย่างการใช้งาน

### 🚗 Vehicle Control
- `manual_control.py` - ควบคุมรถด้วย keyboard (มี HUD monitoring)
- `automatic_control.py` - รถขับอัตโนมัติ (มี HUD)
- `manual_control_steeringwheel.py` - ควบคุมด้วย steering wheel
- `manual_control_carsim.py` - รวมกับ CarSim
- `manual_control_chrono.py` - รวมกับ Chrono
- `vehicle_physics.py` - ทดสอบ physics ของรถ
- `vehicle_gallery.py` - แสดงรถทั้งหมด

### 📡 Sensors & Visualization
- `visualize_multiple_sensors.py` - **แสดง sensors หลายตัวพร้อมกัน (Camera, LiDAR, Semantic LiDAR)**
- `lidar_to_camera.py` - Project LiDAR points ลงบน camera image
- `open3d_lidar.py` - **3D LiDAR visualization แบบ real-time**
- `sensor_synchronization.py` - Synchronize หลาย sensors
- `bounding_boxes.py` - แสดง bounding boxes
- `client_bounding_boxes.py` - Bounding boxes แบบ client-side
- `draw_skeleton.py` - แสดง skeleton ของ pedestrians

### 🎮 Simulation Control
- `tutorial.py` - Tutorial พื้นฐาน
- `synchronous_mode.py` - Synchronous mode example
- `no_rendering_mode.py` - No rendering mode (performance)
- `tutorial_gbuffer.py` - G-buffer tutorial
- `dynamic_weather.py` - เปลี่ยนสภาพอากาศ
- `generate_traffic.py` - สร้าง traffic

### 📹 Recording & Replay
- `start_recording.py` - บันทึก simulation
- `start_replaying.py` - เล่น simulation ที่บันทึกไว้
- `show_recorder_file_info.py` - ดูข้อมูลไฟล์ที่บันทึก
- `show_recorder_collisions.py` - แสดง collisions
- `show_recorder_actors_blocked.py` - แสดง actors ที่ถูก block

### 🔬 Advanced Features
- `V2XDemo.py` - V2X (Vehicle-to-Everything) demo
- `invertedai_traffic.py` - Inverted AI traffic
- `carla_cosmos_gen.py` - Cosmos generation
- `get_component_test.py` - ทดสอบ components
- `test_addsecondvx.py` - ทดสอบ V2X

### 📦 Specialized
- `rss/manual_control_rss.py` - RSS (Responsibility-Sensitive Safety)
- `rss/rss_sensor.py` - RSS sensor
- `rss/rss_visualization.py` - RSS visualization
- `ros2/ros2_native.py` - ROS2 integration
- `nvidia/cosmos/` - NVIDIA Cosmos tools
- `nvidia/nurec/` - NVIDIA NuRec tools

## 📁 PythonAPI/util/ - Utilities

- `config.py` - **Configuration และ inspection tools (มี monitoring)**
- `test_connection.py` - ทดสอบการเชื่อมต่อ CARLA
- `performance_benchmark.py` - **Benchmark performance**
- `check_map.py` - ตรวจสอบ map
- `check_lidar_bb.py` - ตรวจสอบ LiDAR bounding boxes
- `check_collisions_substepping.py` - ตรวจสอบ collisions
- `check_raycast_sensors_determinism.py` - ตรวจสอบ raycast sensors
- `vehicle_physics_tester.py` - ทดสอบ vehicle physics
- `extract_spawn_points.py` - Extract spawn points
- `lane_explorer.py` - สำรวจ lanes
- `osm_to_xodr.py` - แปลง OSM เป็น OpenDRIVE
- `apply_texture.py` - ใช้ texture
- `environment.py` - Environment utilities
- `raycast_sensor_testing.py` - ทดสอบ raycast sensors

## 📁 PythonAPI/carla/ - CARLA API

- `agents/` - Navigation agents
- `dist/` - Wheel files สำหรับติดตั้ง
- `requirements.txt` - Dependencies
- `scene_layout.py` - Scene layout utilities

## 🎯 ไฟล์ที่น่าสนใจสำหรับ Monitoring

1. **`PythonAPI/examples/manual_control.py`** - มี HUD แสดงข้อมูล real-time:
   - FPS (Server & Client)
   - Vehicle info
   - Speed, Location, Compass
   - IMU data (Accelerometer, Gyroscope)
   - GNSS coordinates
   - Collision history
   - Nearby vehicles

2. **`PythonAPI/util/config.py`** - Inspection tool:
   - Simulation status
   - Actors count
   - Map & weather info
   - Frame rate

3. **`PythonAPI/examples/visualize_multiple_sensors.py`** - Multi-sensor visualization:
   - 4 RGB Cameras
   - LiDAR
   - Semantic LiDAR
   - Real-time display

4. **`PythonAPI/util/performance_benchmark.py`** - Performance monitoring:
   - FPS metrics
   - System specs
   - Benchmark results

## 🚀 วิธีใช้งาน

```bash
cd /home/a/Desktop/CARLA_0.9.16
source venv/bin/activate

# Monitoring & Inspection
python PythonAPI/util/config.py --inspect
python PythonAPI/util/test_connection.py

# Manual Control with HUD
python PythonAPI/examples/manual_control.py

# Multi-sensor visualization
python PythonAPI/examples/visualize_multiple_sensors.py --sync

# Performance benchmark
python PythonAPI/util/performance_benchmark.py --help
```

