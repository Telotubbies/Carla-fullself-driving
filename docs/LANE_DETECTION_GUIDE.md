# Lane Detection Guide

## Overview

ระบบ Lane Detection สำหรับปรับปรุงคุณภาพของ features ที่ส่งไป LSTM

## 2 วิธี

### 1. Fine-tune ResNet (แนะนำ)
- Fine-tune ResNet ด้วย lane labels
- Features จะจับ lane ได้ดีขึ้น
- ใช้ได้กับ LSTM โดยตรง

### 2. Lane Detection Model แยก (U-Net)
- Train U-Net สำหรับ lane detection
- ส่ง lane features แยกไป LSTM
- ใช้ร่วมกับ ResNet features

## การใช้งาน

### Step 1: สร้าง Lane Labels

```bash
# ใช้ CARLA's lane detection
python3 -c "
from perception.lane_detector import create_lane_labels_from_carla
import carla

client = carla.Client('localhost', 2000)
world = client.get_world()
vehicle = world.get_actors().filter('vehicle.*')[0]

create_lane_labels_from_carla(
    'data/autopilot_20260208_150902/images',
    'data/autopilot_20260208_150902/lane_masks',
    world, vehicle
)
"
```

### Step 2: Train Models

```bash
# Train ทั้ง U-Net และ Fine-tune ResNet
./scripts/training/train_lane_detection.sh data/autopilot_20260208_150902
```

หรือแยก:

```bash
# Train U-Net
python3 training/train_lane_unet.py \
    --images-dir data/autopilot_20260208_150902/images \
    --masks-dir data/autopilot_20260208_150902/lane_masks \
    --epochs 20

# Fine-tune ResNet
python3 training/finetune_resnet_lane.py \
    --data-dir data/autopilot_20260208_150902 \
    --epochs 10
```

### Step 3: ใช้ใน Inference

#### Option A: ใช้ Fine-tuned ResNet

```python
# ใน config.yaml
perception:
  model_path: data/autopilot_20260208_150902/resnet_lane_model/resnet_lane_final.pth
  freeze_backbone: false  # Unfreeze เพื่อใช้ fine-tuned weights
```

#### Option B: ใช้ Lane Detector + LSTM

```python
from perception.lane_detector import LaneDetector
from temporal import LSTMPredictor

# Initialize
lane_detector = LaneDetector(model_path="lane_unet_final.pth")
lstm = LSTMPredictor(use_lane_features=True, lane_feature_dim=128)

# Inference
resnet_features = resnet_encoder.encode(image)
lane_mask, lane_features = lane_detector.detect_lanes(image)
prediction = lstm.predict(resnet_features, lane_features)
```

## ผลลัพธ์ที่คาดหวัง

- **Fine-tuned ResNet**: Features จับ lane ได้ดีขึ้น → LSTM เรียนรู้ได้ดีขึ้น
- **Lane Detector**: Lane features แยก → LSTM มีข้อมูล lane โดยตรง

## Tips

1. **Fine-tune ResNet**: ใช้ได้เลย ไม่ต้องแก้โค้ดมาก
2. **Lane Detector**: ต้องแก้ LSTM input ให้รับ lane features
3. **Combined**: ใช้ทั้งสองแบบพร้อมกันได้

