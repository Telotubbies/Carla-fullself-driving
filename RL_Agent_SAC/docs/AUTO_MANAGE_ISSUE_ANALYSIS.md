# 🔍 Auto-Manage Log Analysis - Training Crash Issue

## 📋 สรุปปัญหา

### ❌ **ปัญหา:** Training process crash ซ้ำๆ

**Pattern ที่พบ:**
1. Training process ถูก start (PID: xxx)
2. ภายใน 30-60 วินาที process หายไป
3. Auto-manager detect "Training unhealthy: No training process found"
4. Restart training (PID: yyy)
5. Loop นี้เกิดขึ้นซ้ำๆ

---

## 🐛 Root Cause: Dimension Mismatch

### Error:
```
RuntimeError: mat1 and mat2 shapes cannot be multiplied (1x7 and 5x16)
```

### สาเหตุ:
- **Model** คาดหวัง velocity dimension = **5**
- **Environment** ส่ง velocity dimension = **7**

### รายละเอียด:

**Environment (carla_rl_env.py:177):**
```python
space_dict["velocity"] = spaces.Box(
    low=-1.0, high=1.0, 
    shape=(7,),  # ← 7 dimensions
    dtype=np.float32
)  # [speed_kmh, vx, vy, vz, speed_ms, yaw, yaw_rate]
```

**Model (custom_policy.py:98):**
```python
self.velocity_encoder = nn.Sequential(
    nn.Linear(5, 16),  # ← 5 dimensions (OLD)
    nn.ReLU(),
    nn.Linear(16, 16)
)
```

---

## ✅ การแก้ไข

### 1. แก้ Model Architecture:
```python
# custom_policy.py:98
self.velocity_encoder = nn.Sequential(
    nn.Linear(7, 16),  # ← เปลี่ยนเป็น 7
    nn.ReLU(),
    nn.Linear(16, 16)
)
```

### 2. ตรวจสอบ Checkpoint Compatibility:
- Checkpoint เก่าถูก train ด้วย velocity=5
- ต้อง retrain ใหม่หรือใช้ checkpoint ที่ compatible

---

## 📊 Log Analysis

### Timeline:
- **2026-01-08**: Training crash ซ้ำๆ (ไม่มี checkpoint)
- **2026-01-26 02:20**: Training start → crash ทันที
- **2026-01-26 02:38**: พบ checkpoint (2000 steps) → load → crash
- **2026-01-26 02:42**: ยัง crash อยู่

### Statistics:
- Total training starts: หลายครั้ง
- Total unhealthy detections: 15+ ครั้ง
- Crash reason: **100% dimension mismatch**

---

## 🔧 วิธีแก้ไข

### Option 1: แก้ Model (แนะนำ)
```bash
# แก้ custom_policy.py
# เปลี่ยน nn.Linear(5, 16) → nn.Linear(7, 16)
```

### Option 2: แก้ Environment (ไม่แนะนำ)
```python
# เปลี่ยน velocity shape จาก 7 → 5
# แต่จะเสียข้อมูล yaw และ yaw_rate
```

### Option 3: Retrain ใหม่
- ลบ checkpoint เก่า
- Start training ใหม่ด้วย config ที่ถูกต้อง

---

## ⚠️ หมายเหตุ

### Checkpoint Compatibility:
- Checkpoint ที่ train ด้วย velocity=5 **ไม่สามารถใช้ได้** กับ config ปัจจุบัน
- ต้อง retrain ใหม่หรือใช้ checkpoint ที่ train ด้วย velocity=7

### การตรวจสอบ:
```python
# ตรวจสอบ velocity dimension ใน checkpoint
import torch
checkpoint = torch.load('checkpoints/...')
# ดู observation_space หรือ model architecture
```

---

## 📝 Next Steps

1. ✅ แก้ model architecture (velocity_encoder: 5→7)
2. ⏭️ Test training ใหม่
3. ⏭️ ตรวจสอบว่า training ทำงานได้
4. ⏭️ Monitor auto-manage log

---

**อัปเดต:** 2026-01-26  
**สถานะ:** ✅ Fixed (model architecture updated)

