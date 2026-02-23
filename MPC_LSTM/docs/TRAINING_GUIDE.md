# 🎓 Training Guide - CARLA LSTM-MPC

## สรุป Training Pipeline

```
STEP 1: เก็บ Data (Autopilot)
  ↓
STEP 2: Extract Features (ResNet)
  ↓
STEP 3: Train LSTM
  ↓
STEP 4: ใช้ LSTM + MPC (Inference)
```

## STEP 1: เก็บ Dataset ด้วย Autopilot

### รัน Data Collection

```bash
cd /home/a/Desktop/CARLA_0.9.16/carla_lstm_mpc_project

# เริ่ม CARLA ก่อน (ใน terminal แยก)
cd /home/a/Desktop/CARLA_0.9.16
./CarlaUE4.sh

# รัน data collection (ใน terminal อีกอัน)
cd /home/a/Desktop/CARLA_0.9.16/carla_lstm_mpc_project
export HSA_OVERRIDE_GFX_VERSION=11.0.0
export CARLA_DIR=/home/a/Desktop/CARLA_0.9.16
CARLA_EGG=$(find "$CARLA_DIR/PythonAPI/carla/dist" -name "carla-*.egg" 2>/dev/null | head -1)
export PYTHONPATH="$CARLA_EGG:$PYTHONPATH"

python3 training/collect_autopilot_data.py --frames 20000
```

### ข้อมูลที่เก็บ

- **Images**: `data/autopilot_TIMESTAMP/images/`
- **CSV**: `data/autopilot_TIMESTAMP/data.csv`
  - Columns: step, image_path, x, y, yaw, velocity, steering, throttle, brake

### คำแนะนำ

- **จำนวน Frames**: อย่างน้อย 20,000-50,000 frames
- **เวลา**: ~10-30 นาที (ขึ้นกับ FPS)
- **Autopilot**: ใช้ CARLA autopilot อัตโนมัติ

## STEP 2: Extract Features ด้วย ResNet

### รัน Feature Extraction

```bash
python3 training/extract_features.py data/autopilot_20260208_120000/
```

### Output

- `features.npy`: Feature vectors (N, 512)
- `valid_indices.npy`: Valid indices
- `data_valid.csv`: Valid data

### หมายเหตุ

- ใช้ ResNet18 pretrained (ImageNet)
- ไม่ต้อง train ResNet (freeze weights)
- Feature dimension: 512

## STEP 3: Train LSTM

### รัน Training

```bash
python3 training/train_lstm.py data/autopilot_20260208_120000/ \
    --output data/autopilot_20260208_120000/lstm_model \
    --epochs 50 \
    --batch-size 64 \
    --lr 0.001
```

### Parameters

- `--sequence-length`: 10 (default)
- `--hidden-size`: 256 (default)
- `--num-layers`: 2 (default)
- `--epochs`: 50 (default)
- `--batch-size`: 64 (default)
- `--lr`: 0.001 (default)

### Output

- `best_model.pth`: Trained model
- `training_history.json`: Training curves

## STEP 4: ใช้ LSTM + MPC (Inference)

### โหลด Trained Model

แก้ไข `main.py` หรือสร้าง wrapper:

```python
from training.load_trained_model import load_trained_lstm

# Load model
model_path = "data/autopilot_20260208_120000/lstm_model/best_model.pth"
trained_lstm, state_mean, state_std = load_trained_lstm(model_path)

# Replace default LSTM
system.lstm_predictor = trained_lstm
```

### รัน Inference

```bash
python3 main.py --mode inference
```

## สรุป Training Checklist

- [ ] STEP 1: เก็บ Data (20k-50k frames)
- [ ] STEP 2: Extract Features
- [ ] STEP 3: Train LSTM (50 epochs)
- [ ] STEP 4: Test Inference
- [ ] Evaluate Performance

## Tips

1. **Data Quality**: เก็บข้อมูลในหลายๆ สถานการณ์ (ถนนตรง, เลี้ยว, ทางแยก)
2. **Normalization**: LSTM จะ normalize states อัตโนมัติ
3. **Validation**: ใช้ 80/20 split (train/val)
4. **Early Stopping**: Model จะ save best model อัตโนมัติ
5. **GPU**: ใช้ ROCm/CUDA สำหรับ training (เร็วกว่า CPU มาก)

## Troubleshooting

### Data Collection ช้า
- ลด resolution
- ปิด visualization
- ใช้ SSD สำหรับเก็บข้อมูล

### Training Loss ไม่ลด
- ตรวจสอบ data quality
- เพิ่ม epochs
- ลด learning rate
- เพิ่ม batch size

### Inference ไม่ดี
- ตรวจสอบว่าใช้ trained model
- ตรวจสอบ normalization
- เพิ่ม training data

