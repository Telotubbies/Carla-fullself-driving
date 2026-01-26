# Dataset Attributes สำหรับรายงาน

เอกสารนี้สรุป attributes ทั้งหมดที่เก็บใน dataset/replay buffer และข้อมูลที่ track ระหว่าง training

---

## 1. REPLAY BUFFER (DictReplayBuffer)

### 1.1 Observations (Dict)

#### `vision`
- **Type**: `np.ndarray`
- **Shape**: `(sequence_length=4, height=90, width=160, channels=4)`
- **Content**: RGB + Depth images (stacked frames)
- **Data Type**: `float32`, normalized `[0, 1]`
- **Description**: ภาพจาก RGB camera และ Depth camera ที่ stack กัน 4 frames

#### `gps`
- **Type**: `np.ndarray`
- **Shape**: `(3,)`
- **Content**: `[x, y, z]` GPS location
- **Data Type**: `float32`
- **Description**: ตำแหน่ง GPS ของรถ

#### `goal`
- **Type**: `np.ndarray`
- **Shape**: `(3,)`
- **Content**: `[x, y, z]` Goal location
- **Data Type**: `float32`
- **Description**: ตำแหน่งเป้าหมาย

#### `distance_to_goal`
- **Type**: `np.ndarray`
- **Shape**: `(1,)`
- **Content**: Distance to goal (meters)
- **Data Type**: `float32`
- **Description**: ระยะทางถึงเป้าหมาย (เมตร)

#### `waypoint`
- **Type**: `np.ndarray`
- **Shape**: `(8,)`
- **Content**: `[dx, dy, dz, dist, curve, left_lane, right_lane, angle]`
  - `dx, dy, dz`: Waypoint direction vector
  - `dist`: Distance to waypoint
  - `curve`: Road curvature
  - `left_lane, right_lane`: Lane availability (0 or 1)
  - `angle`: Steering angle
- **Data Type**: `float32`, normalized `[-1, 1]`
- **Description**: ข้อมูล waypoint จาก lane detector

#### `velocity`
- **Type**: `np.ndarray`
- **Shape**: `(5,)`
- **Content**: `[speed_norm, vx, vy, vz, speed_ms_norm]`
  - `speed_norm`: Normalized speed
  - `vx, vy, vz`: Velocity components (m/s)
  - `speed_ms_norm`: Speed in m/s (normalized)
- **Data Type**: `float32`, normalized `[-1, 1]`
- **Description**: ข้อมูลความเร็วของรถ

### 1.2 Actions
- **Type**: `np.ndarray`
- **Shape**: `(3,)`
- **Content**: `[steering, throttle, brake]`
  - `steering`: `float [-1.0, 1.0]` - มุมพวงมาลัย
  - `throttle`: `float [0.0, 1.0]` - ทอก
  - `brake`: `float [0.0, 1.0]` - เบรก
- **Data Type**: `float32`

### 1.3 Rewards
- **Type**: `np.ndarray`
- **Shape**: `(1,)`
- **Content**: Reward value (float)
- **Data Type**: `float32`
- **Description**: Reward ที่คำนวณจาก reward function

### 1.4 Next Observations
- **Type**: `Dict` (same structure as observations)
- **Description**: Observation ของ state ถัดไป

### 1.5 Dones
- **Type**: `np.ndarray`
- **Shape**: `(1,)`
- **Content**: Episode done flag (bool)
- **Data Type**: `bool`
- **Description**: บอกว่า episode จบหรือไม่

---

## 2. INFO DICT (จาก environment.step())

ข้อมูลที่ return จาก `env.step()` ใน `info` dictionary:

- **`collision`**: `bool` - เกิด collision หรือไม่
- **`speed`**: `float` - ความเร็ว (km/h)
- **`total_distance`**: `float` - ระยะทางรวมที่เดินทาง (meters)
- **`lane_keeping_ratio`**: `float` - อัตราส่วนการอยู่ในเลน (0-1)
- **`step_duration`**: `float` - เวลาที่ใช้ในแต่ละ step (seconds)
- **`step_count`**: `int` - จำนวน steps ใน episode
- **`performance_warning`**: `bool` - Step ช้าเกินไป (>100ms)
- **`episode`**: `dict` (เมื่อ episode จบ)
  - `r`: `float` - Episode reward
  - `l`: `int` - Episode length (steps)
- **`terminal_observation`**: `dict` (เมื่อ episode จบ)
- **`TimeLimit.truncated`**: `bool` - Episode ถูกตัดด้วย time limit

---

## 3. EPISODE METRICS (ที่ track อยู่)

Metrics ที่เก็บไว้ใน `episode_metrics`:

- **`episode_reward`**: `float` - Reward รวมของ episode
- **`episode_length`**: `int` - จำนวน steps ใน episode
- **`total_distance`**: `float` - ระยะทางรวม (meters)
- **`lane_keeping_time`**: `int` - จำนวน steps ที่อยู่ในเลน
- **`total_steps`**: `int` - จำนวน steps รวม
- **`collision`**: `bool` - เกิด collision หรือไม่
- **`goal_reached`**: `bool` - ถึง goal หรือไม่
- **`distance_to_goal`**: `float` - ระยะทางถึง goal (meters)
- **`route_completion`**: `float` - อัตราส่วนการเดินทางเสร็จ
- **`jerk_sum`**: `float` - ผลรวมของ jerk (ความเร่งของการเร่ง)
- **`speed_variance`**: `float` - ความแปรปรวนของความเร็ว
- **`infractions`**: `int` - จำนวนการทำผิดกฎ
- **`speeds`**: `list` - รายการความเร็วในแต่ละ step

---

## 4. TRAINING METRICS (จาก logs)

Metrics ที่ log ระหว่าง training:

- **`Step`**: `int` - Training step number
- **`Episode reward`**: `float` - Reward ของ episode
- **`Episode length`**: `int` - จำนวน steps ใน episode
- **`Buffer size`**: `int` - จำนวน transitions ใน replay buffer
- **`GPU Memory`**: `float` - GPU memory usage (%)
- **`GPU Utilization`**: `float` - GPU utilization (%)
- **`Mixed Mode`**: `bool` - Mixed device mode เปิดหรือไม่
- **`CPU Batch Ratio`**: `float` - อัตราส่วน batches ที่ process บน CPU
- **`GPU Batches`**: `int` - จำนวน batches ที่ process บน GPU
- **`CPU Batches`**: `int` - จำนวน batches ที่ process บน CPU
- **`OOM Events`**: `int` - จำนวน OOM events
- **`CPU Fallbacks`**: `int` - จำนวน CPU fallback events

---

## 5. ACTION ATTRIBUTES

- **`steering`**: `float` - มุมพวงมาลัย (-1.0 ถึง 1.0)
- **`throttle`**: `float` - ทอก (0.0 ถึง 1.0)
- **`brake`**: `float` - เบรก (0.0 ถึง 1.0)

---

## 6. REWARD COMPONENTS

Components ที่ใช้ในการคำนวณ reward:

- **`lane_center_reward`**: `float` - Reward สำหรับการอยู่กลางเลน
- **`speed_reward`**: `float` - Reward สำหรับความเร็ว (50-90 km/h)
- **`progress_reward`**: `float` - Reward สำหรับความก้าวหน้า
- **`smooth_steering_reward`**: `float` - Reward สำหรับการเลี้ยวที่ราบรื่น
- **`goal_reached_reward`**: `float` - Reward สำหรับการถึง goal (100.0)
- **`collision_penalty`**: `float` - Penalty สำหรับ collision (-50.0)
- **`off_lane_penalty`**: `float` - Penalty สำหรับการออกนอกเลน (-10.0)
- **`low_speed_penalty`**: `float` - Penalty สำหรับความเร็วต่ำ (<40 km/h)
- **`high_speed_penalty`**: `float` - Penalty สำหรับความเร็วสูง (>100 km/h)
- **`goal_distance_reward`**: `float` - Penalty สำหรับระยะทางถึง goal (-0.1 * distance)

---

## 7. ตัวอย่างการใช้งาน

### 7.1 ดึงข้อมูลจาก Replay Buffer

```python
# ใน Stable Baselines3
buffer = model.replay_buffer

# ดึง sample
sample = buffer.sample(batch_size=32)

# Access observations
obs = sample.observations  # Dict with keys: vision, gps, goal, etc.
actions = sample.actions
rewards = sample.rewards
next_obs = sample.next_observations
dones = sample.dones
```

### 7.2 ดึงข้อมูลจาก Info Dict

```python
obs, reward, done, truncated, info = env.step(action)

# Access info
collision = info['collision']
speed = info['speed']
total_distance = info['total_distance']
lane_keeping_ratio = info['lane_keeping_ratio']

# Episode info (เมื่อ episode จบ)
if 'episode' in info:
    episode_reward = info['episode']['r']
    episode_length = info['episode']['l']
```

### 7.3 ดึงข้อมูลจาก Logs

```python
import re

with open('logs/sac_training_*.log', 'r') as f:
    content = f.read()

# Extract episode rewards
episode_rewards = re.findall(r'Episode reward: ([\d.-]+)', content)

# Extract step counts
steps = re.findall(r'Callback: Step (\d+)', content)

# Extract GPU memory
gpu_memory = re.findall(r'GPU memory: ([\d.]+)%', content)
```

---

## 8. สรุปสำหรับรายงาน

### 8.1 Performance Metrics
- Episode reward (mean, max, min)
- Episode length (mean, max, min)
- Collision rate
- Lane keeping ratio
- Average speed
- Total distance traveled

### 8.2 Training Metrics
- Training steps
- Buffer size
- GPU/CPU utilization
- Mixed mode statistics
- OOM events

### 8.3 Action Statistics
- Steering distribution
- Throttle distribution
- Brake distribution

### 8.4 Reward Components
- Lane center reward
- Speed reward
- Progress reward
- Collision penalty
- Off-lane penalty

---

## หมายเหตุ

- ข้อมูลทั้งหมดเก็บใน `DictReplayBuffer` ของ Stable Baselines3
- Observation space เป็น `Dict` ที่มี keys: `vision`, `gps`, `goal`, `distance_to_goal`, `waypoint`, `velocity`
- Action space เป็น `Box` shape `(3,)` สำหรับ `[steering, throttle, brake]`
- Info dict จะมีข้อมูลเพิ่มเติมเมื่อ episode จบ (เช่น `episode`, `terminal_observation`)

