# RL Agent SAC - Soft Actor-Critic Implementation

โปรเจกต์นี้เป็น implementation ของ SAC (Soft Actor-Critic) สำหรับการ train RL agent ใน CARLA environment

## 🎯 ความแตกต่างจากโปรเจกต์ PPO

### SAC (Soft Actor-Critic)
- **Off-policy algorithm**: เรียนรู้จาก experiences ที่เก็บไว้ใน replay buffer
- **Sample efficiency**: ใช้ข้อมูลได้อย่างมีประสิทธิภาพมากกว่า on-policy algorithms
- **Stable learning**: มีความเสถียรมากกว่า PPO ในบางกรณี
- **Continuous actions**: เหมาะกับ continuous action space

### PPO (Proximal Policy Optimization)
- **On-policy algorithm**: เรียนรู้จาก experiences ที่ collect ใหม่เท่านั้น
- **Simplicity**: ง่ายกว่าในการ implement และ tune
- **Current status**: กำลังทำงานได้ดี (reward กำลังดีขึ้น)

## 📁 โครงสร้างโปรเจกต์

```
RL_Agent_SAC/
├── carla_env/          # CARLA environment wrapper
├── models/             # Policy networks และ vision encoders
├── training/           # Training scripts
│   └── train_sac.py   # Main SAC training script
├── config/             # Configuration files
│   └── sac_config.yaml # SAC configuration
├── utils/              # Utility functions
├── checkpoints/        # Saved models
├── logs/               # Training logs
└── requirements.txt   # Python dependencies
```

## 🚀 การใช้งาน

### 1. Setup Environment

```bash
cd /home/a/Desktop/CARLA_0.9.16/RL_Agent_SAC

# Create virtual environment (optional)
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### 2. เริ่ม Training

```bash
# Basic training
python training/train_sac.py --config config/sac_config.yaml

# Resume from checkpoint
python training/train_sac.py --config config/sac_config.yaml --resume checkpoints/checkpoint/rl_model_100000_steps.zip

# Multiple environments (may not provide much benefit for SAC)
python training/train_sac.py --config config/sac_config.yaml --num-envs 2
```

### 3. Monitor Training

```bash
# View logs
tail -f logs/sac_training_*.log

# TensorBoard
tensorboard --logdir logs/tensorboard
```

## ⚙️ Configuration

แก้ไข `config/sac_config.yaml` เพื่อปรับ hyperparameters:

### SAC-Specific Parameters

```yaml
training:
  sac:
    learning_rate: 0.0003        # Learning rate
    buffer_size: 100000          # Replay buffer size
    learning_starts: 1000        # Steps before training starts
    batch_size: 256              # Batch size
    tau: 0.005                   # Soft update coefficient
    gamma: 0.99                  # Discount factor
    ent_coef: 'auto'             # Entropy coefficient (auto-tuned)
```

### Mixed Device (GPU/CPU Load Balancing)

```yaml
device:
  use_gpu: true
  gpu_id: 0
  # Mixed device settings (GPU/CPU load balancing)
  use_mixed_device: true              # Enable automatic GPU/CPU load balancing
  gpu_memory_threshold: 0.85         # Use CPU if GPU memory > 85%
  gpu_util_threshold: 0.90           # Use CPU if GPU utilization > 90%
  mixed_device_check_interval: 100   # Check GPU status every N steps
```

**Mixed Device Feature:**
- เมื่อ GPU memory หรือ utilization สูงเกิน threshold ระบบจะ:
  - ลด batch size อัตโนมัติเพื่อป้องกัน OOM
  - Clear GPU cache เมื่อจำเป็น
  - Monitor และ log GPU usage
- ช่วยให้ training ไม่หยุดเมื่อ GPU ใช้เต็ม
- ปรับ threshold ตามความต้องการ (0.85 = 85%)

## 📊 เปรียบเทียบ PPO vs SAC

| Feature | PPO | SAC |
|---------|-----|-----|
| **Algorithm Type** | On-policy | Off-policy |
| **Sample Efficiency** | Lower | Higher (20-40% better) |
| **Stability** | Good | Very Good |
| **Implementation** | Simpler | More complex |
| **Replay Buffer** | Not used | Required |
| **Best For** | Simple tasks, quick iteration | Complex tasks, sample efficiency |

## 🔧 Customization

### ใช้ Prioritized Replay Buffer

แก้ไข `config/sac_config.yaml`:

```yaml
training:
  prioritized_replay:
    enabled: true
    capacity: 100000
    alpha: 0.6
    beta: 0.4
```

### ใช้ Hindsight Experience Replay (HER)

แก้ไข `config/sac_config.yaml`:

```yaml
training:
  hindsight_replay:
    enabled: true
```

## 📝 Notes

1. **SAC เป็น off-policy**: สามารถเรียนรู้จาก experiences เก่าได้ ทำให้ sample efficient กว่า
2. **Replay buffer**: ต้องมีขนาดใหญ่พอ (100k+ transitions) เพื่อเก็บ diverse experiences
3. **Learning starts**: ต้อง collect experiences ก่อนเริ่ม training (default: 1000 steps)
4. **Entropy tuning**: SAC ใช้ automatic entropy tuning (`ent_coef: 'auto'`) เพื่อ balance exploration/exploitation

## 🐛 Troubleshooting

### GPU ไม่ทำงาน
- ตรวจสอบ `device.use_gpu: true` ใน config
- ตรวจสอบว่า PyTorch รองรับ GPU ของคุณ

### Training ช้า
- ลด `buffer_size` ถ้า memory ไม่พอ
- ลด `batch_size` ถ้า GPU memory ไม่พอ
- ใช้ `num_envs: 1` (SAC ไม่ได้ประโยชน์มากจาก multiple envs)
- เปิดใช้ mixed device (`use_mixed_device: true`) เพื่อให้ระบบจัดการ GPU memory อัตโนมัติ

### GPU Memory เต็ม (OOM)
- เปิดใช้ mixed device: `use_mixed_device: true`
- ลด `batch_size` ใน config
- ลด `buffer_size` ถ้าไม่จำเป็น
- ปรับ `gpu_memory_threshold` ให้ต่ำลง (เช่น 0.75) เพื่อให้เริ่มใช้ CPU ก่อน

### Reward ไม่ดีขึ้น
- ปรับ `learning_rate` (ลอง 0.0001 หรือ 0.0005)
- เพิ่ม `buffer_size` เพื่อเก็บ experiences มากขึ้น
- ตรวจสอบ reward shaping ใน config

## 📚 References

- [SAC Paper](https://arxiv.org/abs/1801.01290)
- [Stable-Baselines3 SAC Documentation](https://stable-baselines3.readthedocs.io/en/master/modules/sac.html)

