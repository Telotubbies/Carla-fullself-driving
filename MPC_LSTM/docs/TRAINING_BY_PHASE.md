# Training by Phase Guide

## Overview

Script สำหรับ train AI models ตาม MASTER FLOW phases โดยใช้ code เดิมที่มีอยู่แล้ว

## Usage

```bash
# Run training for all phases
./scripts/training/train_by_phase.sh
```

## Phases

### PHASE 2: UNet Training

**Input**: RGB images + Lane masks  
**Output**: Segmentation model  
**Script**: `training/train_lane_unet.py`

**Requirements**:
- Images in `data/autopilot_*/images/`
- Lane masks in `data/autopilot_*/lane_masks/` (created from CARLA)

**Training Parameters**:
- Epochs: 30
- Batch size: 8
- Learning rate: 0.0001

**Validation**:
- Inference speed measurement
- IoU and pixel accuracy (if validation data available)

### PHASE 3: ResNet Fine-tuning

**Input**: RGB images + Lane masks  
**Output**: Fine-tuned ResNet for lane detection  
**Script**: `training/finetune_resnet_lane.py`

**Requirements**:
- Images in `data/autopilot_*/images/`
- Lane masks in `data/autopilot_*/lane_masks/`

**Training Parameters**:
- Epochs: 20
- Batch size: 16
- Learning rate: 0.001

**Output**: Better features for LSTM

### PHASE 3.5: Feature Extraction

**Input**: Images + Fine-tuned ResNet (optional)  
**Output**: Feature vectors (`features.npy`)  
**Script**: `training/extract_features.py`

**Uses**: Fine-tuned ResNet if available, otherwise pretrained ResNet

### PHASE 4: LSTM Training

**Input**: Feature sequences + Vehicle states  
**Output**: Trajectory prediction model  
**Script**: `training/train_lstm.py`

**Requirements**:
- `features.npy` in data directory
- `data.csv` with vehicle states

**Training Parameters**:
- Epochs: 150
- Batch size: 64
- Learning rate: 0.001
- Sequence length: 10
- Hidden size: 256
- Layers: 2
- Advanced features: Attention, Advanced Loss, Gradient Clipping, Early Stopping

**Validation**:
- Training history saved
- Can validate using `temporal/lstm_validator.py`

## Training Flow

```
1. Check/Create Lane Masks (from CARLA)
   ↓
2. Train UNet (Phase 2)
   ↓
3. Fine-tune ResNet (Phase 3)
   ↓
4. Extract Features (Phase 3.5)
   ↓
5. Train LSTM (Phase 4)
   ↓
6. Update config.yaml
```

## Manual Training

### Train UNet Only

```bash
python3 training/train_lane_unet.py \
    --images-dir data/autopilot_20260208_150902/images \
    --masks-dir data/autopilot_20260208_150902/lane_masks \
    --epochs 30 \
    --batch-size 8 \
    --lr 0.0001
```

### Fine-tune ResNet Only

```bash
python3 training/finetune_resnet_lane.py \
    --data-dir data/autopilot_20260208_150902 \
    --masks-dir data/autopilot_20260208_150902/lane_masks \
    --epochs 20 \
    --batch-size 16 \
    --lr 0.001
```

### Extract Features Only

```bash
python3 training/extract_features.py \
    --data-dir data/autopilot_20260208_150902 \
    --preprocess \
    --resnet-model data/autopilot_20260208_150902/resnet_lane_model/resnet_lane_final.pth
```

### Train LSTM Only

```bash
python3 training/train_lstm.py data/autopilot_20260208_150902 \
    --epochs 150 \
    --batch-size 64 \
    --lr 0.001 \
    --sequence-length 10 \
    --hidden-size 256 \
    --num-layers 2 \
    --use-attention \
    --use-advanced-loss \
    --gradient-clip 1.0 \
    --early-stopping 20
```

## Validation After Training

### Validate UNet

```python
from perception.unet_validator import UNetValidator
from perception.lane_detector import LaneUNet
from utils.device_utils import get_device
import torch

model = LaneUNet().to(get_device())
model.load_state_dict(torch.load('data/autopilot_20260208_150902/lane_unet_model/lane_unet_final.pth'))
model.eval()

# Measure speed
speed = UNetValidator.measure_inference_speed(model)
print(f"Inference: {speed['mean_ms']:.2f} ms ({speed['fps']:.2f} FPS)")

# Validate on dataset (if masks available)
results = UNetValidator.validate_on_dataset(
    model,
    Path('data/autopilot_20260208_150902/images'),
    Path('data/autopilot_20260208_150902/lane_masks')
)
UNetValidator.print_validation_results(results)
```

### Validate LSTM

```python
from temporal.lstm_validator import LSTMValidator
from pathlib import Path

results = LSTMValidator.validate_on_logs(
    Path('logs/predictions_*.csv'),
    Path('logs/trajectory_*.csv')
)
LSTMValidator.print_validation_results(results)
```

## Notes

- Training จะใช้ data directory ล่าสุด (`data/autopilot_*`)
- ถ้า model มีอยู่แล้วจะ skip training
- Lane masks จะถูกสร้างจาก CARLA ถ้ายังไม่มี
- Config จะถูก update อัตโนมัติหลัง training เสร็จ

## Troubleshooting

### No lane masks
- ต้องมี CARLA running
- Run: `python3 training/create_lane_labels.py --images-dir <dir> --output-dir <dir>`

### Training fails
- ตรวจสอบว่า data มีเพียงพอ (อย่างน้อย 1000 samples)
- ตรวจสอบ GPU/CPU memory
- ลด batch size ถ้า memory ไม่พอ

### Features not found
- Run feature extraction ก่อน train LSTM
- ตรวจสอบว่า `features.npy` มีอยู่ใน data directory

