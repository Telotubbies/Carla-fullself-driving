# Status Monitoring Guide

## 📊 Centralized Status System

ระบบมี centralized status logger ที่เก็บข้อมูลทั้งหมดไว้ในไฟล์เดียว

### ไฟล์ Status
- **Location**: `logs/system_status.json`
- **Format**: JSON
- **Auto-update**: อัปเดตอัตโนมัติระหว่าง training

### วิธีดู Status

#### 1. ใช้ Script (แนะนำ)
```bash
# ดู status summary
python3 scripts/view_status.py

# ดู status แบบ JSON
python3 scripts/view_status.py --json

# อัปเดต status
python3 utils/status_logger.py --scan --summary
```

#### 2. ใช้ Shell Script
```bash
./scripts/check_status.sh
```

#### 3. ดู JSON โดยตรง
```bash
cat logs/system_status.json | jq
```

---

## 📋 Status Structure

### Data Pipeline
- `image_count`: จำนวน images
- `mask_count`: จำนวน lane masks
- `feature_size`: ขนาด features file
- `data_dir`: Path ของ data directory

### Models
แต่ละ model มี:
- `exists`: มี model หรือไม่
- `size`: ขนาดไฟล์ (MB)
- `trained_at`: เวลาที่ train
- `val_loss`: Validation loss
- `path`: Path ของ model

### Training
แต่ละ training job มี:
- `status`: running/completed/failed
- `current_epoch`: Epoch ปัจจุบัน
- `total_epochs`: จำนวน epochs ทั้งหมด
- `train_loss`: Train loss ล่าสุด
- `val_loss`: Validation loss ล่าสุด
- `best_val_loss`: Best validation loss
- `learning_rate`: Learning rate ปัจจุบัน

### Metrics
- Metrics ต่างๆ ที่ track

### System
- `config_loaded`: Config โหลดแล้วหรือไม่
- `lstm_path`: Path ของ LSTM model ใน config
- `resnet_path`: Path ของ ResNet model ใน config

---

## 🔄 Auto-Update

Status จะอัปเดตอัตโนมัติเมื่อ:
- Training scripts ทำงาน (LSTM, ResNet)
- เรียก `scan_system_status()`
- เรียก `python3 utils/status_logger.py --scan`

---

## 📈 Training History

แต่ละ model จะบันทึก training history ใน:
- `data/autopilot_*/lstm_model/training_history.json`
- `data/autopilot_*/resnet_lane_model/training_history.json`

History รวม:
- `train_losses`: List ของ train loss ทุก epoch
- `val_losses`: List ของ validation loss ทุก epoch
- `learning_rates`: Learning rate ทุก epoch (LSTM)
- `grad_norms`: Gradient norm ทุก epoch (LSTM)
- `best_epoch`: Epoch ที่มี best validation loss
- `best_val_loss`: Best validation loss
- `config`: Training configuration

---

## 💡 Usage Examples

### Check if training is running
```bash
python3 scripts/view_status.py | grep TRAINING
```

### Check model status
```bash
python3 scripts/view_status.py | grep MODELS
```

### Monitor training progress
```bash
watch -n 5 'python3 scripts/view_status.py'
```

### Get specific metric
```bash
python3 -c "from utils.status_logger import StatusLogger; s=StatusLogger(); print(s.get_status()['models']['resnet_lane']['val_loss'])"
```

