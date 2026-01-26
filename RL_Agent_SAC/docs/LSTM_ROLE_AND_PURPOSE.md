# 🧠 LSTM ใช้ทำอะไร?

**Date**: January 26, 2026  
**Status**: ✅ **Current Implementation**

---

## 🎯 คำตอบสั้นๆ

**LSTM ใช้สำหรับ Temporal Encoding** - การเข้ารหัสข้อมูลตามเวลา (sequence) เพื่อให้ agent เข้าใจ motion patterns และ temporal context

---

## 📊 LSTM ในระบบ CARLA RL Agent

### หน้าที่หลัก

LSTM ทำหน้าที่ **3 อย่างหลัก**:

1. **🔄 Temporal Encoding** - เข้ารหัสข้อมูลตามเวลา
2. **🎬 Motion Understanding** - เข้าใจการเคลื่อนไหว
3. **💾 Sequence Memory** - จดจำลำดับเหตุการณ์

---

## 🔄 1. Temporal Encoding (การเข้ารหัสข้อมูลตามเวลา)

### Input/Output

```
Input:
  - Shape: (batch_size, 4, 512)
  - 4 frames × 512 features per frame
  - Features มาจาก ResNet-18 encoder

Output:
  - Shape: (batch_size, 256)
  - 256-dimensional temporal features
  - จาก last hidden state ของ LSTM
```

### Process Flow

```
Frame 1 (t-3) → ResNet-18 → 512 features
Frame 2 (t-2) → ResNet-18 → 512 features
Frame 3 (t-1) → ResNet-18 → 512 features
Frame 4 (t-0) → ResNet-18 → 512 features
                    ↓
        Stack: (4, 512)
                    ↓
              LSTM Encoder
                    ↓
        Output: (256) temporal features
```

### Code Implementation

```python
# models/vision_encoder.py

class TemporalEncoder(nn.Module):
    def __init__(
        self,
        input_size: int,      # 512 (from ResNet-18)
        hidden_size: int = 256,
        num_layers: int = 2,
        dropout: float = 0.1
    ):
        super().__init__()
        self.lstm = nn.LSTM(
            input_size,        # 512
            hidden_size,       # 256
            num_layers,       # 2
            batch_first=True,
            dropout=dropout if num_layers > 1 else 0.0
        )
    
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x shape: (batch_size, 4, 512)
        lstm_out, (hidden, cell) = self.lstm(x)
        # Return last hidden state: (batch_size, 256)
        return hidden[-1]
```

---

## 🎬 2. Motion Understanding (เข้าใจการเคลื่อนไหว)

### ทำไมต้องเข้าใจ Motion?

ใน autonomous driving, **motion information** สำคัญมาก:

- ✅ **รถกำลังเลี้ยว**: ต้องรู้ว่ารถกำลังเลี้ยวไปทางไหน
- ✅ **รถกำลังเร่ง/ชะลอ**: ต้องรู้ว่ารถกำลังเร่งหรือชะลอ
- ✅ **วัตถุกำลังเคลื่อนที่**: ต้องรู้ว่ารถคันอื่นกำลังทำอะไร
- ✅ **สภาพแวดล้อมเปลี่ยนแปลง**: ต้องรู้ว่าสภาพแวดล้อมเปลี่ยนไปอย่างไร

### ตัวอย่างการทำงาน

#### ตัวอย่างที่ 1: การเลี้ยว

```
Frame 1 (t-3): รถอยู่ตรงกลางเลน
Frame 2 (t-2): รถเริ่มเลี้ยวซ้าย (steering = -0.1)
Frame 3 (t-1): รถเลี้ยวซ้ายมากขึ้น (steering = -0.3)
Frame 4 (t-0): รถกำลังเลี้ยวซ้าย (steering = -0.5)
        ↓
LSTM: "รถกำลังเลี้ยวซ้าย" → temporal features
        ↓
SAC Policy: ตัดสินใจ action ต่อไป (อาจจะเลี้ยวต่อหรือกลับเลน)
```

#### ตัวอย่างที่ 2: การเร่ง/ชะลอ

```
Frame 1 (t-3): รถอยู่ห่างจากรถคันหน้า 50m
Frame 2 (t-2): รถอยู่ห่างจากรถคันหน้า 40m
Frame 3 (t-1): รถอยู่ห่างจากรถคันหน้า 30m
Frame 4 (t-0): รถอยู่ห่างจากรถคันหน้า 20m
        ↓
LSTM: "รถกำลังเข้าใกล้รถคันหน้า" → temporal features
        ↓
SAC Policy: ตัดสินใจชะลอ (brake) หรือเปลี่ยนเลน
```

#### ตัวอย่างที่ 3: การหลบหลีก

```
Frame 1 (t-3): มีรถคันอื่นอยู่ทางซ้าย
Frame 2 (t-2): รถคันอื่นเริ่มเคลื่อนที่
Frame 3 (t-1): รถคันอื่นกำลังเข้าใกล้
Frame 4 (t-0): รถคันอื่นอยู่ใกล้มาก
        ↓
LSTM: "มีรถคันอื่นกำลังเข้าใกล้" → temporal features
        ↓
SAC Policy: ตัดสินใจหลบหลีกหรือชะลอ
```

---

## 💾 3. Sequence Memory (จดจำลำดับเหตุการณ์)

### LSTM Memory Mechanism

LSTM มี **memory cells** ที่สามารถ:

- ✅ **จดจำ**: เก็บข้อมูลจาก frames ก่อนหน้า
- ✅ **ลืม**: ลืมข้อมูลที่ไม่สำคัญ
- ✅ **อัพเดท**: อัพเดทข้อมูลใหม่

### Architecture

```
LSTM Cell Structure:
  - Input Gate: ตัดสินใจว่าข้อมูลไหนควรเก็บ
  - Forget Gate: ตัดสินใจว่าข้อมูลไหนควรลืม
  - Output Gate: ตัดสินใจว่าข้อมูลไหนควรส่งออก
  - Cell State: เก็บข้อมูลระยะยาว
  - Hidden State: เก็บข้อมูลระยะสั้น
```

### Configuration

```yaml
# config/sac_config.yaml

network:
  temporal:
    hidden_size: 256      # ขนาดของ hidden state
    num_layers: 2         # จำนวน layers
    type: lstm
```

### ทำไมต้อง 2 Layers?

- **Layer 1**: เรียนรู้ low-level temporal patterns (motion, speed)
- **Layer 2**: เรียนรู้ high-level temporal patterns (behavior, intent)

---

## 📐 Data Flow: จาก Frames ถึง LSTM

### Step 1: Frame Stacking

```python
# carla_env/carla_rl_env.py

self.sequence_length = 4  # 4 frames
self.rgb_buffer = deque(maxlen=self.sequence_length)
self.depth_buffer = deque(maxlen=self.sequence_length)

# ในแต่ละ step:
self.rgb_buffer.append(rgb_frame)      # เพิ่ม frame ใหม่
self.depth_buffer.append(depth_frame)  # เพิ่ม frame ใหม่

# Stack frames:
rgb_stack = np.stack(self.rgb_buffer, axis=0)    # (4, 90, 160, 3)
depth_stack = np.stack(self.depth_buffer, axis=0) # (4, 90, 160, 1)
vision = np.concatenate([rgb_stack, depth_stack], axis=-1)  # (4, 90, 160, 4)
```

### Step 2: ResNet-18 Encoding

```python
# models/vision_encoder.py

# แต่ละ frame ผ่าน ResNet-18:
# Input: (batch, 4, 90, 160, 4)
# Output: (batch, 4, 512)  # 4 frames × 512 features
```

### Step 3: LSTM Temporal Encoding

```python
# models/vision_encoder.py

# LSTM process sequence:
# Input: (batch, 4, 512)
# Output: (batch, 256)  # temporal features
```

### Step 4: Feature Fusion

```python
# models/custom_policy.py

# Combine features:
temporal_features = 256D  # จาก LSTM
gps_features = 16D        # จาก GPS encoder
goal_features = 16D      # จาก Goal encoder
waypoint_features = 16D   # จาก Waypoint encoder
velocity_features = 16D   # จาก Velocity encoder

# Total: 320D
fused_features = concat([temporal, gps, goal, waypoint, velocity])
```

### Step 5: SAC Policy

```python
# SAC Policy Network:
# Input: 320D fused features
# Output: Actions (steering, throttle, brake)
```

---

## 💡 ทำไมต้องใช้ LSTM?

### ❌ ปัญหาถ้าไม่มี LSTM

1. **เห็นแค่ frame เดียว**:
   - ไม่รู้ว่ารถกำลังทำอะไร
   - ไม่รู้ว่าสภาพแวดล้อมเปลี่ยนไปอย่างไร
   - ตัดสินใจผิดพลาดได้ง่าย

2. **ไม่มี temporal context**:
   - ไม่รู้ว่ารถกำลังเลี้ยวหรือไม่
   - ไม่รู้ว่ารถกำลังเร่งหรือชะลอ
   - ไม่รู้ว่าวัตถุกำลังเคลื่อนที่หรือไม่

3. **ตัดสินใจผิดพลาด**:
   - อาจจะตัดสินใจเลี้ยวผิดทิศทาง
   - อาจจะไม่รู้ว่าต้องชะลอ
   - อาจจะไม่รู้ว่าต้องหลบหลีก

### ✅ ข้อดีเมื่อมี LSTM

1. **เห็น 4 frames**:
   - เห็นการเปลี่ยนแปลงของสภาพแวดล้อม
   - เห็น motion patterns
   - ตัดสินใจได้แม่นยำขึ้น

2. **มี temporal context**:
   - รู้ว่ารถกำลังเลี้ยวไปทางไหน
   - รู้ว่ารถกำลังเร่งหรือชะลอ
   - รู้ว่าวัตถุกำลังเคลื่อนที่หรือไม่

3. **ตัดสินใจถูกต้อง**:
   - ตัดสินใจเลี้ยวถูกทิศทาง
   - รู้ว่าต้องชะลอเมื่อเข้าใกล้รถคันหน้า
   - รู้ว่าต้องหลบหลีกเมื่อมีรถคันอื่น

---

## 🔬 Technical Details

### LSTM Architecture

```python
LSTM Configuration:
  - Input size: 512 (from ResNet-18)
  - Hidden size: 256
  - Number of layers: 2
  - Dropout: 0.1 (if layers > 1)
  - Batch first: True
  - Bidirectional: False (unidirectional)
```

### Memory Cells

- **Cell State (C_t)**: เก็บข้อมูลระยะยาว
- **Hidden State (h_t)**: เก็บข้อมูลระยะสั้น
- **Gates**:
  - Input Gate (i_t): ตัดสินใจว่าข้อมูลไหนควรเก็บ
  - Forget Gate (f_t): ตัดสินใจว่าข้อมูลไหนควรลืม
  - Output Gate (o_t): ตัดสินใจว่าข้อมูลไหนควรส่งออก

### Training

- LSTM ถูก train **ร่วมกับ SAC policy**
- Gradient flow จาก SAC loss → LSTM
- LSTM เรียนรู้ temporal patterns ที่ช่วย policy

---

## 📊 Summary

| Aspect | Description |
|--------|-------------|
| **หน้าที่หลัก** | Temporal Encoding, Motion Understanding, Sequence Memory |
| **Input** | 4 frames × 512 features (from ResNet-18) |
| **Output** | 256-dimensional temporal features |
| **Architecture** | 2 layers, 256 hidden units |
| **ทำไมต้องใช้** | เพื่อให้ agent เข้าใจ temporal context และ motion patterns |
| **Location** | `models/vision_encoder.py` (class `TemporalEncoder`) |

---

## ✅ Conclusion

**LSTM ใช้สำหรับ**:
- ✅ **Temporal Encoding**: เข้ารหัสข้อมูลตามเวลา
- ✅ **Motion Understanding**: เข้าใจการเคลื่อนไหว
- ✅ **Sequence Memory**: จดจำลำดับเหตุการณ์

**ผลลัพธ์**: Agent ที่เข้าใจ temporal context และตัดสินใจได้แม่นยำขึ้น

---

**Status**: ✅ **LSTM is actively used for temporal encoding**  
**Performance**: ✅ **Improves agent's understanding of motion and temporal patterns**

