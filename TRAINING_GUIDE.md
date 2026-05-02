# 🎓 Training Guide with Guidelines

**วันที่:** 10 เมษายน 2026  
**สถานะ:** ✅ พร้อมเทรนด้วย Guidelines

---

## 🎯 ภาพรวม

ระบบนี้มี **Training Guidelines** ที่กำหนดพารามิเตอร์การเรียนรู้อย่างชัดเจน:

### พารามิเตอร์ที่กำหนด:

1. **ความเร็ว (Speed)**
   - เป้าหมาย: 30 km/h
   - ถนนตรง: 30 km/h
   - โค้งเบา: 25 km/h
   - โค้งแหลม: 15 km/h

2. **การเลี้ยว (Steering)**
   - เลี้ยวนุ่มนวล: ไม่เกิน 0.2 ต่อ step
   - เลี้ยวเบา: < 0.3
   - เลี้ยวแหลม: > 0.7

3. **การอยู่ในเลน (Lane Keeping)**
   - ยอมรับได้: ±0.5m จากศูนย์กลาง
   - เตือน: ±1.0m
   - วิกฤต: ±1.5m

4. **ความปลอดภัย (Safety)**
   - ระยะปลอดภัย: 5m
   - โทษการชน: -200
   - โทษเกือบชน: -10

5. **ความนุ่มนวล (Smoothness)**
   - การเร่ง/เบรกสบาย: 2-3 m/s²
   - การเร่ง/เบรกแรง: 4-6 m/s²

---

## 🚀 วิธีเริ่มเทรน

### Option 1: เทรนด้วย Random Actions (เริ่มต้น)

```bash
cd /home/supawich/Desktop/carla_sac_ros2_training
source venv/bin/activate
python scripts/train_with_guidelines.py --episodes 100
```

### Option 2: เทรนด้วย Expert Controller (แนะนำ)

```bash
python scripts/train_with_guidelines.py --episodes 100 --expert
```

---

## 📊 Reward Components

ระบบคำนวณ reward จาก 6 components:

### 1. Progress Reward (ความคืบหน้า)
```python
# ได้รางวัลจากการเคลื่อนที่ไปข้างหน้า
- ดี: > 0.5 m/step
- พอใช้: 0.1-0.5 m/step
- แย่: < 0.1 m/step (stuck)
- โทษ: ถอยหลัง -1.0
```

### 2. Lane Keeping Reward (การอยู่ในเลน)
```python
# โบนัสสำหรับอยู่กึ่งกลางเลน
- Perfect (< 0.1m): +0.5
- ยอมรับได้ (< 0.5m): -0.1 * offset
- เตือน (< 1.0m): -0.5 * offset
- วิกฤต (< 1.5m): -1.0 * offset
- ออกนอกเลน (> 1.5m): -2.0 * offset
```

### 3. Speed Tracking Reward (การควบคุมความเร็ว)
```python
# ตรงตามเป้าหมาย ±5 km/h
- ในช่วง: +0.5 * (1 - error/tolerance)
- นอกช่วง: -0.1 * (error - tolerance)
- เร็วเกินไป: -1.0
```

### 4. Smooth Control Reward (ความนุ่มนวล)
```python
# การควบคุมนุ่มนวล
- Steering นุ่มนวล: +0.1
- Steering กระตุก: -0.2
- Acceleration สบาย: +0.1
- Acceleration แรง: -0.3
```

### 5. Heading Alignment (การจัดทิศทาง)
```python
# ทิศทางตรงตามเส้นทาง
- ดี (< 5°): +0.2
- พอใช้ (< 15°): -0.01 * error
- แย่ (< 30°): -0.5 * error
- แย่มาก (> 30°): -2.0
```

### 6. Collision Penalty (โทษการชน)
```python
# ชนแล้วจบ episode ทันที
- ชน: -200.0
```

---

## 📚 Curriculum Learning (3 Stages)

### Stage 1: Basic Control (0-500 episodes)
**เป้าหมาย:** เรียนรู้การควบคุมพื้นฐาน

**สภาพแวดล้อม:**
- Spawn points: [0, 1, 2] (ถนนตรง)
- Traffic: ไม่มี
- ความเร็ว: 30 km/h
- โค้ง: ไม่มี

**Reward Weights:**
- Lane keeping: 1.0 (สำคัญที่สุด)
- Progress: 0.5
- Speed: 0.3
- Smoothness: 0.1
- Heading: 0.2

**Success Criteria:**
- Lane keeping accuracy: > 80%
- Collision rate: < 10%

---

### Stage 2: Navigation (500-1500 episodes)
**เป้าหมาย:** เรียนรู้การเลี้ยวและตามเส้นทาง

**สภาพแวดล้อม:**
- Spawn points: [0-5] (มีโค้ง)
- Traffic: 10%
- ความเร็ว: 30 km/h
- โค้ง: มี

**Reward Weights:**
- Progress: 1.0 (สำคัญที่สุด)
- Lane keeping: 0.5
- Speed: 0.3
- Smoothness: 0.2
- Heading: 0.3

**Success Criteria:**
- Path following: > 70%
- Collision rate: < 15%

---

### Stage 3: Complex Scenarios (1500+ episodes)
**เป้าหมาย:** จัดการสถานการณ์ซับซ้อน

**สภาพแวดล้อม:**
- Spawn points: ทั้งหมด
- Traffic: 30%
- ความเร็ว: 30 km/h
- โค้ง: มี
- รถอื่น: มี

**Reward Weights:**
- Progress: 0.8
- Lane keeping: 0.4
- Speed: 0.4
- Smoothness: 0.3
- Heading: 0.2
- Safety: 0.5

**Success Criteria:**
- Completion rate: > 60%
- Collision rate: < 20%

---

## 🎮 Episode Termination

Episode จะจบเมื่อ:

1. **Collision** - ชนสิ่งกีดขวาง
2. **Max Steps** - ครบ 1000 steps
3. **Lane Deviation** - ออกนอกเลน > 2.0m
4. **Heading Error** - ทิศทางผิด > 45°
5. **Stuck** - ความเร็วต่ำกว่า 1 km/h นาน 100 steps

---

## 📈 Monitoring & Logging

### MLflow Tracking
ทุก episode จะถูก log ไป MLflow:

```python
# Metrics ที่ track:
- episode_reward
- episode_length
- distance_traveled
- collision_count
- success_rate
- avg_speed
- avg_lane_deviation
- reward_components (แยกตาม component)
```

**ดู MLflow UI:** http://localhost:5000

### Tensorboard
```bash
# เปิด Tensorboard
tensorboard --logdir=data/tensorboard --port=6006
```

**ดู Tensorboard:** http://localhost:6006

---

## 🔧 Configuration Files

### 1. Training Guidelines
**File:** `config/training_guidelines.yaml`

กำหนดทุกพารามิเตอร์:
- Speed limits
- Steering constraints
- Lane keeping thresholds
- Safety distances
- Reward weights
- Curriculum stages

### 2. Enhanced Rewards
**File:** `src/carla_gym_env/enhanced_rewards.py`

คำนวณ reward ตาม guidelines:
- Progress reward
- Lane keeping reward
- Speed tracking reward
- Smooth control reward
- Heading alignment reward
- Collision penalty

### 3. Training Script
**File:** `scripts/train_with_guidelines.py`

Main training loop:
- Curriculum management
- Episode execution
- Metrics logging
- MLflow tracking

---

## 📊 Expected Results

### After 100 Episodes (Stage 1):
- Lane keeping: ~60-70%
- Success rate: ~40-50%
- Avg distance: ~50-100m

### After 500 Episodes (Stage 1 Complete):
- Lane keeping: >80%
- Success rate: >70%
- Avg distance: >150m

### After 1500 Episodes (Stage 2 Complete):
- Path following: >70%
- Success rate: >60%
- Can handle curves

### After 3000 Episodes (Stage 3 Complete):
- Full scenario completion: >60%
- Safe driving: >80%
- Can handle traffic

---

## 🎯 Tips for Better Training

### 1. Start with Expert
```bash
# ใช้ expert controller ก่อน 100-200 episodes
python scripts/train_with_guidelines.py --episodes 200 --expert
```

### 2. Monitor Metrics
- ดู MLflow UI เพื่อ compare runs
- ดู Tensorboard สำหรับ real-time metrics
- ตรวจสอบ reward components

### 3. Adjust Guidelines
แก้ไข `config/training_guidelines.yaml`:
- ปรับ reward weights
- เปลี่ยน speed limits
- ปรับ success criteria

### 4. Save Checkpoints
```python
# Checkpoint จะถูก save อัตโนมัติทุก 100 episodes
# ที่ data/curriculum_ep{episode}.pkl
```

---

## 🐛 Troubleshooting

### ปัญหา: Agent ไม่เรียนรู้
**แก้ไข:**
- เพิ่ม exploration
- ลด reward weights ที่ขัดแย้งกัน
- ใช้ expert controller ก่อน

### ปัญหา: Agent ชนบ่อย
**แก้ไข:**
- เพิ่ม collision penalty
- เพิ่ม safety distance reward
- ลด speed limit

### ปัญหา: Agent ออกนอกเลน
**แก้ไข:**
- เพิ่ม lane keeping weight
- ลด acceptable deviation
- เพิ่ม heading alignment weight

### ปัญหา: Agent ขับกระตุก
**แก้ไข:**
- เพิ่ม smooth control weight
- ลด max steering/throttle change
- เพิ่ม smoothness bonus

---

## 📝 Example Training Session

```bash
# Terminal 1: CARLA Server
cd /home/supawich/Desktop/CARLA_0.9.16
./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2000

# Terminal 2: MLflow UI
cd /home/supawich/Desktop/carla_sac_ros2_training
./scripts/start_mlflow_ui.sh

# Terminal 3: Training
cd /home/supawich/Desktop/carla_sac_ros2_training
source venv/bin/activate
python scripts/train_with_guidelines.py --episodes 1000 --expert

# Terminal 4: Tensorboard (optional)
tensorboard --logdir=data/tensorboard --port=6006
```

---

## 🎉 สรุป

**ระบบพร้อมเทรนด้วย Guidelines!**

✅ Training Guidelines กำหนดพารามิเตอร์ชัดเจน  
✅ Enhanced Reward Calculator คำนวณ reward ตาม guidelines  
✅ Curriculum Learning 3 stages  
✅ MLflow Tracking ทุก metric  
✅ Expert Controller สำหรับ warm start  

**เริ่มเทรนเลย:**
```bash
python scripts/train_with_guidelines.py --episodes 100 --expert
```

**ดู Results:**
- MLflow: http://localhost:5000
- Tensorboard: http://localhost:6006

**Happy Training! 🚗💨**
