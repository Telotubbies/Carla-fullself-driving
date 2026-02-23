# 🚗 Quick Start - ระบบ MPC (ไม่ใช้ RL)

## 📋 สถานะปัจจุบัน

✅ **มี Model ที่ train แล้ว:**
- LSTM Model: `data/autopilot_20260208_150902/lstm_model/best_model.pth`
- Config: `config.yaml` (ชี้ไปที่ model แล้ว)

✅ **ระบบพร้อมใช้งาน:**
- MPC Controller: `control/mpc_controller.py`
- LSTM Predictor: `temporal/lstm_predictor.py`
- ResNet Encoder: `perception/resnet_encoder.py`

---

## 🚀 วิธีใช้งาน (3 ขั้นตอน)

### วิธีที่ 1: รัน Inference ทันที (ถ้ามี Model แล้ว)

```bash
# 1. ตรวจสอบว่า CARLA กำลังรันอยู่
pgrep -f "CarlaUE4" || echo "⚠️  CARLA ไม่ได้รัน - ต้องเริ่มก่อน!"

# 2. ถ้า CARLA ไม่ได้รัน ให้เริ่มก่อน
cd /home/a/Desktop/CARLA_0.9.16
./CarlaUE4.sh -quality-level=Low -prefernoloadscreen &
sleep 40  # รอให้ CARLA โหลดเสร็จ

# 3. รัน Inference
cd /home/a/Desktop/CARLA_0.9.16/carla_lstm_mpc_project
python3 main.py --mode inference
```

### วิธีที่ 2: ใช้ Auto Pipeline (แนะนำ)

```bash
cd /home/a/Desktop/CARLA_0.9.16/carla_lstm_mpc_project
./run_complete_pipeline.sh
```

**จะทำอะไร:**
- ✅ ตรวจสอบว่ามี data/model แล้วหรือยัง
- ✅ ถ้ายังไม่มีจะ train ให้อัตโนมัติ
- ✅ สุดท้ายจะรัน inference พร้อม GUI

### วิธีที่ 3: Train ใหม่ (ถ้าต้องการ)

```bash
cd /home/a/Desktop/CARLA_0.9.16/carla_lstm_mpc_project

# 1. เก็บ Data (ถ้ายังไม่มี)
python3 training/collect_autopilot_data.py --frames 20000

# 2. Extract Features
python3 training/extract_features.py --data-dir data/autopilot_XXX --preprocess

# 3. Train LSTM
python3 training/train_lstm.py data/autopilot_XXX --epochs 50

# 4. Update Config
# แก้ไข config.yaml ให้ชี้ไปที่ model ที่ train ใหม่

# 5. Run Inference
python3 main.py --mode inference
```

---

## 🔧 ระบบทำงานอย่างไร

### Flow:
```
Camera Image 
  → ResNet Encoder (Extract Features)
    → LSTM Predictor (Predict Future State)
      → MPC Controller (Compute Control)
        → CARLA Vehicle (Apply Control)
```

### Components:

1. **ResNet Encoder** (`perception/resnet_encoder.py`)
   - รับภาพจาก camera
   - Extract features (512-dim vector)

2. **LSTM Predictor** (`temporal/lstm_predictor.py`)
   - รับ sequence ของ features (10 frames)
   - Predict future state: [x, y, yaw, velocity]

3. **MPC Controller** (`control/mpc_controller.py`)
   - รับ predicted state จาก LSTM
   - สร้าง reference trajectory
   - Optimize control (steering, throttle, brake) ด้วย CasADi
   - ใช้ kinematic bicycle model

---

## ⚙️ Configuration

แก้ไข `config.yaml` เพื่อปรับแต่ง:

```yaml
mpc:
  horizon: 10        # Prediction horizon (steps)
  dt: 0.05           # Time step (seconds)
  vehicle:
    wheelbase: 2.875
    max_steer: 0.5    # Max steering angle (rad)
    max_accel: 3.0    # Max acceleration (m/s²)
  weights:
    lateral_error: 10.0   # Weight for lateral tracking
    heading_error: 5.0    # Weight for heading tracking
    velocity_error: 2.0   # Weight for velocity tracking
```

---

## 🐛 Troubleshooting

### ปัญหา: CARLA ไม่เชื่อมต่อ
```bash
# ตรวจสอบว่า CARLA รันอยู่
pgrep -f "CarlaUE4"

# ถ้าไม่รัน ให้เริ่มใหม่
cd /home/a/Desktop/CARLA_0.9.16
./CarlaUE4.sh -quality-level=Low &
sleep 40
```

### ปัญหา: Model ไม่พบ
```bash
# ตรวจสอบว่า model path ใน config.yaml ถูกต้อง
cat config.yaml | grep trained_model_path

# ถ้าไม่มี model ให้ train ก่อน
python3 training/train_lstm.py data/autopilot_XXX --epochs 50
```

### ปัญหา: Vehicle ไม่เคลื่อนที่
- ตรวจสอบ logs: `logs/main.log`
- ตรวจสอบว่า sequence buffer เต็มแล้วหรือยัง (ต้องรอ 10 frames)
- ลองเพิ่ม throttle ใน config หรือ code

---

## 📊 Monitoring

ดู logs:
```bash
tail -f logs/main.log
```

ดู trajectory/controls:
```bash
ls -lh logs/*.csv
```

---

## 🎯 สรุป

**ถ้ามี Model แล้ว:**
```bash
# 1. เริ่ม CARLA
cd /home/a/Desktop/CARLA_0.9.16
./CarlaUE4.sh -quality-level=Low &

# 2. รอ 40 วินาที

# 3. รัน Inference
cd /home/a/Desktop/CARLA_0.9.16/carla_lstm_mpc_project
python3 main.py --mode inference
```

**ถ้ายังไม่มี Model:**
```bash
./run_complete_pipeline.sh  # จะ train ให้อัตโนมัติ
```

---

## 📝 Notes

- ระบบใช้ **MPC (Model Predictive Control)** ไม่ใช่ RL
- LSTM ทำหน้าที่ predict future state
- MPC ทำหน้าที่ optimize control เพื่อตาม reference trajectory
- ใช้ CasADi สำหรับ optimization
- ใช้ kinematic bicycle model สำหรับ vehicle dynamics

