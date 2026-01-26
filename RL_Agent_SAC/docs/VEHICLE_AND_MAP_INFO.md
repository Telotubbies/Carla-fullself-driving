# 🚗 ข้อมูลการใช้งานรถและ Map

## 📌 คำถามที่ 1: ใช้รถคนเดิมตลอดใช่ไหม?

### ❌ **ไม่ใช่** - ระบบจะ Spawn รถใหม่ทุก Episode (แต่ใช้รุ่นเดิม)

## 📌 คำถามที่ 1.1: ใช้รถรุ่นเดิมตลอดใช่ไหม?

### ✅ **ใช่** - ใช้รถรุ่นเดิมตลอด (Tesla Model 3)

**การตั้งค่าปัจจุบัน:**
- `vehicle_randomization: false` - **ปิดการสุ่มเปลี่ยนรุ่น**
- `blueprint: vehicle.tesla.model3` - ใช้ **Tesla Model 3** ตลอด

**หมายเหตุ:** ระบบมี vehicle_blueprints list ไว้ 13 รุ่น แต่ยังไม่เปิดใช้งาน เพราะ `vehicle_randomization: false`

### 🔍 รายละเอียด:

1. **ทุกครั้งที่ Reset Episode:**
   ```python
   def reset(self, seed: Optional[int] = None, options: Optional[Dict] = None):
       # 1. ทำลายรถและ actors ทั้งหมด
       self._destroy_all_actors()
       
       # 2. Spawn รถใหม่
       if not self._spawn_agent():
           raise RuntimeError("Failed to spawn agent")
   ```

2. **การ Spawn รถ:**
   - ใช้ `_spawn_agent()` method
   - เลือก spawn point แบบสุ่มจาก map
   - Spawn รถใหม่ทุกครั้งที่ reset episode
   - รถเก่าถูก destroy ก่อน spawn ใหม่

3. **เหตุผล:**
   - ✅ หลีกเลี่ยงปัญหา state ที่ผิดพลาด
   - ✅ เริ่มต้น episode ใหม่แบบสะอาด
   - ✅ รองรับ domain randomization (เปลี่ยนรถแบบสุ่ม)

### 📝 Code Location:
- `carla_env/carla_rl_env.py`:
  - `reset()` method (line 565-608)
  - `_spawn_agent()` method (line 354-417)
  - `_destroy_all_actors()` method

---

## 🗺️ คำถามที่ 2: สงสัยเรื่องการเรียนรู้ของรถ มันมี map ใช่ไหม?

### ✅ **ใช่** - ระบบใช้ Map สำหรับ Navigation และ Learning

### 🔍 รายละเอียด:

#### 1. **CARLA Map System:**
```python
# ใช้ CARLA Map API
spawn_points = self.world.get_map().get_spawn_points()
waypoint = self.world.get_map().get_waypoint(vehicle_location)
```

#### 2. **Features ที่ใช้ Map:**

**a) Waypoints (เส้นทาง):**
- ✅ `use_waypoint: true` (default)
- ใช้ `get_waypoint()` เพื่อหา waypoint ที่ใกล้ที่สุด
- ให้ข้อมูล: lane_id, road_id, lane_width, lane_change
- ใช้สำหรับ navigation และ reward calculation

**b) Vision Waypoints (ทางเลือก):**
- ✅ `use_vision_waypoint: false` (optional)
- ใช้ LaneDetector วิเคราะห์ภาพเพื่อหา waypoint
- เรียนรู้จากภาพแทนการใช้ map API โดยตรง

**c) Goal-based Navigation:**
- ✅ `use_goal: true` (default)
- ใช้ spawn_points เพื่อตั้ง goal location
- คำนวณ distance และ angle ไปยัง goal

**d) Spawn Points:**
- ใช้ `get_spawn_points()` เพื่อ:
  - Spawn รถใหม่
  - ตั้ง goal location
  - Spawn obstacles (รถอื่น, คนเดิน)

#### 3. **ข้อมูลที่ Agent เรียนรู้จาก Map:**

```python
# Waypoint Features (8 dimensions)
obs["waypoint"] = [
    lane_id_normalized,      # เลนปัจจุบัน
    road_id_normalized,       # ถนนปัจจุบัน
    lane_width_normalized,   # ความกว้างเลน
    lane_change_left,        # เปลี่ยนเลนซ้ายได้ไหม
    lane_change_right,       # เปลี่ยนเลนขวาได้ไหม
    waypoint_dx,            # ระยะทาง x ไปยัง waypoint
    waypoint_dy,            # ระยะทาง y ไปยัง waypoint
    lane_center_offset      # offset จากกลางเลน
]
```

#### 4. **Reward ที่ใช้ Map:**

```python
# Lane Keeping Reward
wp = self.world.get_map().get_waypoint(vehicle_location)
lane_center = wp.transform.location
distance_to_center = vehicle_location.distance(lane_center)
lane_keeping_reward = -distance_to_center / 10.0
```

### 📝 Code Locations:

1. **Map Usage:**
   - `carla_rl_env.py`:
     - `_get_waypoint_features()` (line 806-840)
     - `_reset_goal()` (line 456-474)
     - `_spawn_agent()` (line 376)
     - `_spawn_obstacles()` (line 317)

2. **Configuration:**
   - `config/sac_config.yaml`:
     ```yaml
     observations:
       use_waypoint: true      # ใช้ waypoint จาก map
       use_vision_waypoint: false  # ใช้ vision-based waypoint
       use_goal: true          # ใช้ goal-based navigation
     ```

3. **Lane Detection (Optional):**
   - `carla_env/lane_detector.py` - Vision-based waypoint detection

---

## 🎯 สรุป

| คำถาม | คำตอบ | รายละเอียด |
|-------|-------|-----------|
| **ใช้รถคนเดิม?** | ❌ ไม่ใช่ | Spawn ใหม่ทุก episode |
| **มี Map?** | ✅ มี | ใช้ CARLA Map API |
| **ใช้ Map ทำอะไร?** | 🗺️ Navigation | Waypoints, Goals, Spawn Points |
| **Agent เรียนรู้จาก Map?** | ✅ ใช่ | Waypoint features, Lane keeping |

---

## 🔧 การปรับแต่ง

### เปลี่ยนพฤติกรรม Spawn รถ:
```yaml
# config/sac_config.yaml
environment:
  vehicle:
    blueprint: "vehicle.tesla.model3"  # เปลี่ยนรถ
  domain_randomization:
    enabled: true
    vehicle_randomization: true  # สุ่มเปลี่ยนรถ
```

### เปิด/ปิด Map Features:
```yaml
observations:
  use_waypoint: true        # ใช้ waypoint จาก map
  use_vision_waypoint: false  # ใช้ vision-based (ไม่ใช้ map)
  use_goal: true           # ใช้ goal navigation
```

---

**อัปเดต:** 2026-01-26  
**ไฟล์ที่เกี่ยวข้อง:**
- `carla_env/carla_rl_env.py`
- `config/sac_config.yaml`
- `carla_env/lane_detector.py`

