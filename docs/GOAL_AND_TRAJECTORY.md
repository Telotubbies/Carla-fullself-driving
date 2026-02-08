# Goal & Trajectory ในระบบ

## 📍 Goal (จุดหมายปลายทาง)

### สถานะปัจจุบัน: ❌ **ยังไม่มี Goal-based Navigation**

ระบบปัจจุบัน:
- **ไม่มี goal location ที่ชัดเจน**
- ใช้ **lane following** แทน goal-based navigation
- รถจะขับตามเลนที่เจอ ไม่ได้มีจุดหมายปลายทาง

### วิธีที่ระบบทำงานตอนนี้:

1. **Lane Following Mode** (ปัจจุบัน):
   - ใช้ `LanePathPlanner` สร้าง reference trajectory จาก lane detection
   - ใช้ CARLA waypoints เพื่อสร้าง path
   - ไม่มี destination/goal location

2. **Reference Trajectory**:
   - สร้างจาก lane center line
   - หรือจาก CARLA waypoints
   - หรือ straight ahead (fallback)

---

## 🛤️ Trajectory (วิถีการเคลื่อนที่)

### ✅ **มี Trajectory หลายประเภท:**

#### 1. **Reference Trajectory** (วิถีอ้างอิง)
- **ที่มา**: `LanePathPlanner.generate_reference_from_lanes()`
- **รูปแบบ**: `(N+1, 4)` → `[x, y, yaw, velocity]`
- **ใช้ใน**: MPC controller เพื่อคำนวณ control
- **แหล่งข้อมูล**:
  - Lane detection (center line, left/right lanes)
  - CARLA waypoints
  - Straight ahead (fallback)

#### 2. **LSTM Predicted Trajectory** (วิถีที่ LSTM ทำนาย)
- **ที่มา**: `LSTMPredictor.predict()` → ทำนาย state ต่อไป
- **รูปแบบ**: `(4,)` → `[x, y, yaw, velocity]` (single point)
- **ใช้ใน**: 
  - ส่งให้ MPC เป็น reference (optional)
  - แสดงผลใน visualization

#### 3. **MPC Horizon Trajectory** (วิถีที่ MPC วางแผน)
- **ที่มา**: `MPCController.get_predicted_trajectory()`
- **รูปแบบ**: `(N+1, 4)` → `[x, y, yaw, velocity]` สำหรับ horizon
- **ใช้ใน**: 
  - แสดงผลใน visualization (เส้นสีม่วง)
  - ตรวจสอบว่า MPC วางแผนอย่างไร

---

## 🔄 Flow ของ Trajectory

```
Camera Image
    ↓
Lane Detection → Lane Mask + Lane Info
    ↓
LanePathPlanner → Reference Trajectory (N+1, 4)
    ↓
MPC Controller → Optimal Control (steering, throttle, brake)
    ↓
Vehicle Control
```

**Parallel Path:**
```
ResNet Features (sequence)
    ↓
LSTM Predictor → Predicted State (4,)
    ↓
(Optional) → MPC Reference
```

---

## 📊 สถานะปัจจุบัน

### ✅ **มี:**
- Reference trajectory generation
- Lane-based path planning
- MPC trajectory prediction
- Visualization ของ trajectories

### ❌ **ยังไม่มี:**
- **Goal-based navigation** (ไม่มี destination)
- **Global path planning** (ไม่มี route planning)
- **Dynamic goal setting** (ไม่สามารถเปลี่ยน goal ได้)

---

## 💡 แนะนำการเพิ่ม Goal-based Navigation

### 1. **เพิ่ม Goal Location ใน Config:**
```yaml
navigation:
  use_goal: true
  goal_location:
    x: 100.0
    y: 200.0
    z: 0.5
  # หรือใช้ spawn point index
  goal_spawn_point: 5
```

### 2. **เพิ่ม Global Planner:**
- ใช้ CARLA's `GlobalRoutePlanner` เพื่อหา route จาก current location ไป goal
- สร้าง waypoint sequence จาก route

### 3. **ปรับ LanePathPlanner:**
- ใช้ global route waypoints แทน local waypoints
- สร้าง reference trajectory ที่มุ่งไป goal

### 4. **เพิ่ม Goal Reaching Logic:**
- ตรวจสอบว่าใกล้ goal แล้วหรือยัง
- เมื่อถึง goal: หยุด หรือตั้ง goal ใหม่

---

## 🎯 สรุป

**Goal**: ❌ ยังไม่มี - ระบบใช้ lane following แทน

**Trajectory**: ✅ มี 3 ประเภท:
1. Reference Trajectory (จาก lane/waypoints)
2. LSTM Predicted Trajectory
3. MPC Horizon Trajectory

**สถานะ**: ระบบทำงานได้ดีสำหรับ lane following แต่ยังไม่มี goal-based navigation

