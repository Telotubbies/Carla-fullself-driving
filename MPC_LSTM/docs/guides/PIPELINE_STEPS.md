# 🚀 Complete Auto Pipeline - ระบบดีๆ

## 📋 STEP-by-STEP

### STEP 1: Collect Data
- เก็บข้อมูล 20,000 frames จาก CARLA autopilot
- เก็บ: Image, State (x, y, yaw, velocity), Control (steering, throttle, brake)

### STEP 2: Create Lane Labels จาก CARLA ⭐ ใหม่!
- ใช้ CARLA map API สร้าง lane masks จริง
- Project waypoints และ lane markings ไปยัง camera view
- เก็บเป็น `lane_masks/*_lane.png`

### STEP 3: Fine-tune ResNet สำหรับ Lane Detection ⭐ ปรับปรุงแล้ว!
**ปรับปรุง:**
- ✅ ใช้ CARLA lane labels จริง (ไม่ใช่ edge detection)
- ✅ DiceLoss + BCE Loss (ดีกว่า BCE อย่างเดียว)
- ✅ Validation set (80/20 split)
- ✅ Data augmentation (horizontal flip, brightness)
- ✅ Learning rate scheduling
- ✅ Save best model based on validation loss

**Output:** `resnet_lane_model/resnet_lane_final.pth`

### STEP 4: Extract Features
- ใช้ fine-tuned ResNet (ถ้ามี) เพื่อ extract features ที่จับ lane ได้ดีขึ้น
- Output: `features.npy`

### STEP 5: Train LSTM
- Train LSTM ด้วย features ที่ดีขึ้น
- Output: `lstm_model/best_model.pth`

### STEP 6: Update Config
- อัพเดท `config.yaml` ด้วย model paths

### STEP 7: Run Inference with GUI
- รัน autonomous driving system พร้อม GUI

## 🚀 วิธีใช้

```bash
./run_complete_pipeline.sh
```

## 💡 ระบบจะ:
- ✅ ตรวจสอบว่ามี data/labels/models แล้วหรือยัง
- ✅ ข้ามขั้นตอนที่ทำแล้ว (smart skip)
- ✅ Train ResNet ให้จับ lane แม่นขึ้น
- ✅ ใช้ fine-tuned ResNet ในการ extract features

## 📊 สรุปการปรับปรุง

### ปัญหาเดิม:
- ❌ Fine-tuning ใช้ edge detection แทน CARLA labels
- ❌ Loss function ไม่เหมาะสม (BCE อย่างเดียว)
- ❌ ไม่มี validation
- ❌ ไม่มี data augmentation

### แก้ไขแล้ว:
- ✅ ใช้ CARLA lane labels จริง
- ✅ DiceLoss + BCE Loss
- ✅ Validation set + best model saving
- ✅ Data augmentation
- ✅ Learning rate scheduling

## 🎯 ผลลัพธ์ที่คาดหวัง

1. **ResNet จะจับ lane ได้แม่นขึ้น** → Features ดีขึ้น
2. **LSTM จะเรียนรู้ได้ดีขึ้น** → Prediction แม่นขึ้น
3. **MPC จะควบคุมได้ดีขึ้น** → รถขับได้ลื่นขึ้น
