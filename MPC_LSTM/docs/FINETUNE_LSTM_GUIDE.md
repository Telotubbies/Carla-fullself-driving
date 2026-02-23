# 🔄 Fine-tune LSTM Guide

## 📋 Overview

Fine-tune LSTM model จาก checkpoint ที่มีอยู่แล้ว เพื่อปรับปรุงผลลัพธ์

**ข้อดี:**
- ✅ ใช้ model ที่ train แล้วเป็นจุดเริ่มต้น
- ✅ Learning rate ต่ำกว่า (0.0001) เพื่อปรับแต่งละเอียด
- ✅ ประหยัดเวลา (ไม่ต้อง train ใหม่ทั้งหมด)
- ✅ ปรับปรุงผลลัพธ์ได้ดีขึ้น

---

## 🚀 วิธีใช้

### วิธีที่ 1: ใช้ Script (แนะนำ)

```bash
cd /home/a/Desktop/CARLA_0.9.16/carla_lstm_mpc_project

# Fine-tune ด้วย default settings
./scripts/training/finetune_lstm.sh

# หรือระบุ parameters
./scripts/training/finetune_lstm.sh \
    data/autopilot_20260208_150902 \
    data/autopilot_20260208_150902/lstm_model/best_model.pth \
    50 \          # epochs
    0.0001        # learning rate
```

### วิธีที่ 2: ใช้ Python Script โดยตรง

```bash
python3 training/finetune_lstm.py \
    data/autopilot_20260208_150902 \
    --checkpoint data/autopilot_20260208_150902/lstm_model/best_model.pth \
    --epochs 50 \
    --lr 0.0001 \
    --batch-size 64
```

---

## ⚙️ Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--checkpoint` | Required | Path to existing model |
| `--epochs` | 50 | Number of epochs |
| `--lr` | 0.0001 | Learning rate (lower than initial training) |
| `--batch-size` | 64 | Batch size |
| `--train-split` | 0.8 | Train/validation split |
| `--gradient-clip` | 1.0 | Gradient clipping value |
| `--early-stopping` | 15 | Early stopping patience |

---

## 📊 Output

หลังจาก fine-tune เสร็จ จะได้:

1. **`best_model.pth`** - Best model based on validation loss
2. **`fine_tuned_model.pth`** - Fine-tuned model (same as best_model)

**Location:** `data/autopilot_XXX/lstm_model/`

---

## 🔧 ใช้ Fine-tuned Model

### Update config.yaml

```yaml
temporal:
  trained_model_path: data/autopilot_20260208_150902/lstm_model/fine_tuned_model.pth
```

### หรือใช้ best_model.pth

```yaml
temporal:
  trained_model_path: data/autopilot_20260208_150902/lstm_model/best_model.pth
```

---

## 💡 Tips

### 1. Learning Rate
- **Initial training**: 0.001
- **Fine-tuning**: 0.0001 (10x lower)
- **Very fine-tuning**: 0.00001 (100x lower)

### 2. Epochs
- **50 epochs**: สำหรับ fine-tuning ปกติ
- **100 epochs**: ถ้าต้องการปรับปรุงมาก
- **20 epochs**: ถ้า model ดีอยู่แล้ว

### 3. Early Stopping
- **15 patience**: Default
- **20 patience**: ถ้าต้องการ train นานขึ้น
- **10 patience**: ถ้าต้องการหยุดเร็ว

### 4. Batch Size
- **64**: Default (ดีสำหรับ GPU)
- **32**: ถ้า GPU memory ไม่พอ
- **128**: ถ้า GPU memory เยอะ

---

## 📈 Monitoring

### ดู Training Progress

```bash
# ดู logs
tail -f logs/training_*.log

# หรือดู output จาก terminal
# จะแสดง Train Loss, Val Loss, Learning Rate ทุก epoch
```

### ตรวจสอบ Model Quality

```bash
# Test model
python3 -c "
from training.load_trained_model import load_trained_lstm
model, mean, std = load_trained_lstm('data/autopilot_20260208_150902/lstm_model/fine_tuned_model.pth')
print('✅ Model loaded successfully')
print(f'State mean: {mean}')
print(f'State std: {std}')
"
```

---

## 🔄 Fine-tune หลายรอบ

ถ้าต้องการ fine-tune ต่อจาก fine-tuned model:

```bash
# Round 1: Fine-tune from original
./scripts/training/finetune_lstm.sh \
    data/autopilot_20260208_150902 \
    data/autopilot_20260208_150902/lstm_model/best_model.pth \
    50 \
    0.0001

# Round 2: Fine-tune from Round 1
./scripts/training/finetune_lstm.sh \
    data/autopilot_20260208_150902 \
    data/autopilot_20260208_150902/lstm_model/fine_tuned_model.pth \
    30 \
    0.00001  # Even lower LR
```

---

## ⚠️ คำเตือน

1. **Backup model เดิม**: Fine-tuning จะ overwrite `best_model.pth`
   ```bash
   cp data/autopilot_XXX/lstm_model/best_model.pth \
      data/autopilot_XXX/lstm_model/best_model_backup.pth
   ```

2. **ตรวจสอบ checkpoint**: ต้องมี checkpoint ก่อน fine-tune
   ```bash
   ls -lh data/autopilot_XXX/lstm_model/best_model.pth
   ```

3. **GPU Memory**: Fine-tuning ใช้ GPU memory เหมือน training ปกติ

---

## 🎯 สรุป

**Fine-tune LSTM เพื่อ:**
- ✅ ปรับปรุงผลลัพธ์จาก model เดิม
- ✅ ประหยัดเวลา (ไม่ต้อง train ใหม่ทั้งหมด)
- ✅ ใช้ learning rate ต่ำเพื่อปรับแต่งละเอียด

**Quick Start:**
```bash
./scripts/training/finetune_lstm.sh
```

