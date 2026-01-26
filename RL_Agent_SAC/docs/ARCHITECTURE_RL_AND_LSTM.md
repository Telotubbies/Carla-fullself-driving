# 🏗️ Architecture: Reinforcement Learning + LSTM

**Date**: January 26, 2026  
**Status**: ✅ **Current Implementation**

---

## 🎯 คำตอบ: ใช้ทั้งสองอย่าง!

ระบบนี้ใช้ **ทั้ง Reinforcement Learning (SAC) และ LSTM** ร่วมกัน โดยแต่ละส่วนมีบทบาทที่แตกต่างกัน

---

## 📊 Architecture Overview

```
┌─────────────────────────────────────────────────────────┐
│                    CARLA Environment                    │
│  (RGB+Depth images, GPS, Goal, Waypoint, Velocity)      │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│              Data Preprocessing Pipeline                 │
│  • Normalize GPS/Goal to [-1, 1]                        │
│  • Normalize Distance to [0, 1]                         │
│  • Data validation & outlier detection                  │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│              Data Augmentation                           │
│  • Color jitter, Gaussian noise, motion blur            │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│         Vision Encoder (ResNet-18)                      │
│  • Input: 4 stacked frames (90x160x4)                   │
│  • Pretrained on ImageNet                               │
│  • Output: 512-dimensional features per frame           │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│         LSTM Temporal Encoder ⭐                        │
│  • Input: 4 frames × 512 features                       │
│  • Architecture: 2 layers, 256 hidden units             │
│  • Purpose: Process temporal sequence                   │
│  • Output: 256-dimensional temporal features           │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│         Feature Fusion                                  │
│  • Vision (LSTM output): 256D                           │
│  • GPS: 16D (normalized)                                │
│  • Goal: 16D (normalized)                                │
│  • Waypoint: 16D                                        │
│  • Velocity: 16D                                        │
│  • Total: 320D                                          │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│         SAC Policy Network ⭐                            │
│  • Actor Network: 320D → [512, 256, 128] → Actions      │
│  • Critic Networks (Q1, Q2): 320D → [512, 256, 128]    │
│  • Value Network: 320D → [512, 256, 128] → Value       │
│  • Algorithm: Soft Actor-Critic (SAC)                   │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│              Actions                                    │
│  • Steering: [-1.0, 1.0]                                │
│  • Throttle: [0.0, 1.0]                                 │
│  • Brake: [0.0, 1.0]                                    │
└─────────────────────────────────────────────────────────┘
```

---

## 🔄 Reinforcement Learning (SAC)

### บทบาท
- **การเรียนรู้ Policy**: เรียนรู้การตัดสินใจที่ดีที่สุดจาก experience
- **Optimization**: Optimize actions เพื่อ maximize cumulative reward
- **Off-policy Learning**: เรียนรู้จาก replay buffer (sample efficient)

### Components

#### 1. **Actor Network** (Policy)
- **Input**: 320D fused features
- **Architecture**: [512, 256, 128] hidden layers
- **Output**: Mean and std for action distribution
- **Purpose**: สร้าง actions (steering, throttle, brake)

#### 2. **Critic Networks** (Q-functions)
- **Two Q-networks**: Q1 และ Q2 (reduces overestimation)
- **Input**: 320D features + actions
- **Architecture**: [512, 256, 128] hidden layers
- **Output**: Q-values (expected future rewards)
- **Purpose**: ประเมิน value ของ state-action pairs

#### 3. **Value Network**
- **Input**: 320D features
- **Architecture**: [512, 256, 128] hidden layers
- **Output**: State value
- **Purpose**: ประเมิน value ของ states

#### 4. **Replay Buffer**
- **Size**: 250,000 transitions
- **Purpose**: เก็บ experiences สำหรับ off-policy learning
- **Sampling**: Random sampling for training

### SAC Algorithm Features

- ✅ **Automatic Entropy Tuning**: ปรับ entropy coefficient อัตโนมัติ
- ✅ **Soft Target Updates**: Stable learning ด้วย tau=0.005
- ✅ **Double Q-learning**: ใช้ 2 Q-networks เพื่อลด overestimation
- ✅ **Off-policy**: เรียนรู้จาก old experiences

---

## 🧠 LSTM (Long Short-Term Memory)

### บทบาท
- **Temporal Encoding**: ประมวลผล sequence ของ vision frames
- **Motion Understanding**: เรียนรู้ motion patterns จาก 4 frames
- **Temporal Context**: จดจำ sequence ของ events

### Architecture

```python
LSTM Configuration:
  - Input size: 512 (from ResNet-18)
  - Hidden size: 256
  - Number of layers: 2
  - Dropout: 0.1 (if layers > 1)
  - Batch first: True
```

### Input/Output

- **Input**: 
  - Shape: `(batch_size, 4, 512)`
  - 4 frames × 512 features per frame
  - จาก ResNet-18 encoder

- **Output**: 
  - Shape: `(batch_size, 256)`
  - 256-dimensional temporal features
  - จาก last hidden state ของ LSTM

### ทำไมต้องใช้ LSTM?

1. **Temporal Information**: 
   - 4 stacked frames ให้ temporal context
   - LSTM จัดการ sequence information

2. **Motion Patterns**:
   - เรียนรู้การเคลื่อนไหวของรถ
   - เรียนรู้การเปลี่ยนแปลงของสภาพแวดล้อม

3. **Memory**:
   - จดจำ events ที่เกิดขึ้นก่อนหน้า
   - ช่วยในการตัดสินใจ

---

## 🔗 Integration: RL + LSTM

### Data Flow

```
1. CARLA Environment
   ↓
2. Data Preprocessing (Normalization, Validation)
   ↓
3. Data Augmentation
   ↓
4. ResNet-18 Vision Encoder
   → Output: 4 frames × 512 features
   ↓
5. LSTM Temporal Encoder ⭐
   → Input: (batch, 4, 512)
   → Output: (batch, 256) temporal features
   ↓
6. Feature Fusion
   → Combine: Vision(256) + GPS(16) + Goal(16) + Waypoint(16) + Velocity(16)
   → Total: 320D
   ↓
7. SAC Policy Network ⭐
   → Actor: 320D → Actions
   → Critic: 320D + Actions → Q-values
   → Value: 320D → State value
   ↓
8. Actions → CARLA Environment
```

### Training Process

1. **Collect Experience**:
   - Agent ทำ action → ได้ observation + reward
   - เก็บใน replay buffer

2. **Train SAC**:
   - Sample batch จาก replay buffer
   - Update Actor, Critic, Value networks
   - ใช้ LSTM features ที่ผ่าน preprocessing แล้ว

3. **LSTM Training**:
   - LSTM ถูก train ร่วมกับ SAC policy
   - Gradient flow จาก SAC loss → LSTM
   - LSTM เรียนรู้ temporal patterns ที่ช่วย policy

---

## 📊 Configuration

### LSTM Settings
```yaml
network:
  temporal:
    hidden_size: 256
    num_layers: 2
    type: lstm
```

### SAC Settings
```yaml
training:
  algorithm: SAC
  sac:
    learning_rate: 0.0003
    buffer_size: 250000
    batch_size: 256
    gamma: 0.99
    tau: 0.005
    ent_coef: 'auto'
```

---

## 💡 ทำไมต้องใช้ทั้งสอง?

### LSTM จำเป็นเพราะ:
- ✅ **Temporal Context**: 4 frames ให้ temporal information
- ✅ **Motion Understanding**: เรียนรู้ motion patterns
- ✅ **Sequence Memory**: จดจำ sequence ของ events

### SAC จำเป็นเพราะ:
- ✅ **Policy Learning**: เรียนรู้การตัดสินใจที่ดีที่สุด
- ✅ **Reward Optimization**: Optimize actions เพื่อ maximize reward
- ✅ **Sample Efficiency**: Off-policy learning จาก replay buffer

### ทำงานร่วมกัน:
- **LSTM**: จัดการ temporal/spatial features
- **SAC**: เรียนรู้ policy จาก features เหล่านั้น
- **Result**: Agent ที่เข้าใจ temporal context และตัดสินใจได้ดี

---

## 🎯 Summary

| Component | Type | Purpose | Location |
|-----------|------|---------|----------|
| **SAC** | Reinforcement Learning | เรียนรู้ policy, optimize actions | `models/sac_policy.py` |
| **LSTM** | Recurrent Neural Network | Temporal encoding ของ vision frames | `models/vision_encoder.py` |
| **ResNet-18** | Convolutional Neural Network | Vision feature extraction | `models/vision_encoder.py` |
| **Replay Buffer** | Data Structure | เก็บ experiences สำหรับ off-policy learning | `stable_baselines3` |

---

## ✅ Conclusion

**ใช้ทั้งสองอย่าง**: 
- **LSTM** สำหรับ temporal encoding
- **SAC (Reinforcement Learning)** สำหรับ policy learning

ทั้งสองทำงานร่วมกันเพื่อสร้าง agent ที่:
- ✅ เข้าใจ temporal context (LSTM)
- ✅ เรียนรู้ policy ที่ดี (SAC)
- ✅ ตัดสินใจได้อย่างชาญฉลาด (RL + LSTM)

---

**Status**: ✅ **Both RL and LSTM are used**  
**Integration**: ✅ **Seamlessly integrated**  
**Performance**: ✅ **Optimized for autonomous driving**

