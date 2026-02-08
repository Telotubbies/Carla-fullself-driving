# ResNet และ Lane Detection

## สถานะปัจจุบัน

### ResNet Encoder
- **Model**: ResNet18
- **Weights**: ImageNet pretrained
- **Status**: Frozen (ไม่ train)
- **Feature dim**: 512

### ปัญหา
- ❌ **ไม่ได้ fine-tune สำหรับ lane detection**
- ⚠️  **Features เป็น general object recognition**
- ⚠️  **อาจไม่จับเส้น (lane) ได้ดี**

## LSTM จะใช้งานได้ไหม?

### ✅ ใช้งานได้ แต่...
- LSTM **สามารถใช้ features จาก ResNet ได้** (features ไหนก็ได้)
- แต่ **คุณภาพขึ้นอยู่กับ feature quality**
- ถ้า ResNet ไม่จับ lane → LSTM ก็เรียนรู้ได้ไม่ดี

### 📊 ผลกระทบ
```
ResNet (ไม่จับ lane) 
  → Features ไม่ดี 
    → LSTM เรียนรู้ได้ไม่ดี 
      → Prediction ไม่แม่น
        → MPC ควบคุมได้ไม่ดี
```

## วิธีแก้ไข

### Option 1: Fine-tune ResNet (แนะนำ)
```python
# Unfreeze ResNet และ train ด้วย lane images
encoder = ResNetEncoder(freeze_backbone=False)
# Train ด้วย CARLA images ที่มี lane labels
```

### Option 2: Lane Detection Model
```python
# ใช้ LaneNet หรือ U-Net สำหรับ lane detection
# แล้วส่ง features ไป LSTM
lane_features = lane_detector(image)
lstm_input = combine(resnet_features, lane_features)
```

### Option 3: Multi-task Learning
```python
# ResNet + Lane detection head
# Train พร้อมกัน: classification + lane detection
```

## สรุป

### ตอนนี้
- ✅ LSTM **ใช้งานได้** กับ ResNet features
- ⚠️  แต่ **คุณภาพอาจไม่ดี** เพราะ ResNet ไม่จับ lane

### แนะนำ
1. **Fine-tune ResNet** ด้วย CARLA lane images
2. หรือใช้ **Lane detection model** แยก
3. หรือ **Multi-task learning** (ResNet + Lane head)

### ผลลัพธ์ที่คาดหวัง
- Fine-tuned ResNet → Better lane features → Better LSTM → Better control

