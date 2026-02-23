# 🚀 Complete Auto Pipeline

สคริปต์สำหรับรันทุกอย่างอัตโนมัติตั้งแต่ต้นจนจบ พร้อม GUI และ MPC

## 📋 สคริปต์ที่ใช้

### 1. `run_complete_auto.sh`
รันทุกขั้นตอนอัตโนมัติ:
- ✅ ตรวจสอบและ collect data (ถ้ายังไม่มี)
- ✅ Extract features (ถ้ายังไม่มี)  
- ✅ Preprocess data (ถ้ายังไม่มี)
- ✅ Train LSTM (ถ้ายังไม่มี)
- ✅ Update config.yaml
- ✅ Run inference with GUI

### 2. `start_carla_and_run.sh`
เริ่ม CARLA และรัน pipeline:
- ✅ เริ่ม CARLA อัตโนมัติ (ถ้ายังไม่รัน)
- ✅ รอให้ CARLA พร้อม
- ✅ รัน complete pipeline

## 🎯 วิธีใช้

### วิธีที่ 1: รันทุกอย่างอัตโนมัติ (แนะนำ)
```bash
cd /home/a/Desktop/CARLA_0.9.16/carla_lstm_mpc_project
./start_carla_and_run.sh
```

### วิธีที่ 2: ถ้า CARLA รันอยู่แล้ว
```bash
cd /home/a/Desktop/CARLA_0.9.16/carla_lstm_mpc_project
./run_complete_auto.sh
```

## 📊 ขั้นตอนที่รันอัตโนมัติ

### STEP 1: Data Collection
- ตรวจสอบว่ามี data หรือไม่
- ถ้าไม่มีหรือไม่พอ (น้อยกว่า 1,000 frames) → collect 20,000 frames

### STEP 2: Feature Extraction
- ตรวจสอบว่ามี features.npy หรือไม่
- ถ้าไม่มี → extract features ด้วย ResNet

### STEP 2.5: Data Preprocessing
- ตรวจสอบว่ามี processed data หรือไม่
- ถ้าไม่มี → preprocess (cleaning, normalization)

### STEP 3: LSTM Training
- ตรวจสอบว่ามี trained model หรือไม่
- ถ้าไม่มี → train LSTM (50 epochs)
- ถ้ากำลัง train อยู่ → รอให้เสร็จ

### STEP 4: Update Config
- อัพเดท config.yaml ด้วย trained model path

### STEP 5: Run Inference with GUI
- เริ่ม autonomous driving system
- แสดง GUI (camera view, trajectory, graphs)
- รัน MPC control

## 🖥️ GUI Features

เมื่อรัน inference จะแสดง:
- ✅ Camera view (real-time)
- ✅ Predicted trajectory (overlay)
- ✅ Current speed
- ✅ Steering value
- ✅ MPC horizon path
- ✅ Live graphs (Speed vs Time, Steering vs Time)

## ⚙️ Requirements

- CARLA 0.9.16 ต้องรันอยู่ (หรือจะเริ่มอัตโนมัติ)
- Python 3.10+
- ROCm/CUDA สำหรับ GPU training
- Pygame สำหรับ GUI

## 🔧 Environment Variables

สคริปต์จะตั้งค่าอัตโนมัติ:
- `HSA_OVERRIDE_GFX_VERSION=11.0.0` (สำหรับ ROCm)
- `AMD_SERIALIZE_KERNEL=3`
- `PYTHONPATH` (CARLA Python API)

## 📝 Logs

- Training logs: `training_log.txt`
- Main logs: `logs/main.log`
- Trajectory logs: `logs/trajectory_*.csv`
- Control logs: `logs/controls_*.csv`

## 🛑 หยุดการทำงาน

กด `Ctrl+C` เพื่อหยุด inference

## ✅ Checklist

ก่อนรัน ตรวจสอบว่า:
- [ ] CARLA 0.9.16 ติดตั้งแล้ว
- [ ] Python dependencies ติดตั้งแล้ว (`pip install -r requirements.txt`)
- [ ] GPU driver ติดตั้งแล้ว (สำหรับ ROCm/CUDA)
- [ ] มีพื้นที่ว่างพอสำหรับ data และ models

## 🚀 Quick Start

```bash
# 1. ไปที่ project directory
cd /home/a/Desktop/CARLA_0.9.16/carla_lstm_mpc_project

# 2. รันทุกอย่างอัตโนมัติ
./start_carla_and_run.sh

# 3. รอให้ GUI เปิดขึ้นมา
# 4. ดูรถขับอัตโนมัติ!
```

## 📞 Troubleshooting

### CARLA ไม่เริ่ม
```bash
# เริ่ม CARLA ด้วยมือ
cd /home/a/Desktop/CARLA_0.9.16
./CarlaUE4.sh
```

### Training ช้า
- ตรวจสอบว่าใช้ GPU หรือไม่
- ลด epochs หรือ batch size

### GUI ไม่แสดง
- ตรวจสอบว่า pygame ติดตั้งแล้ว
- ตรวจสอบว่า display server ทำงานอยู่

