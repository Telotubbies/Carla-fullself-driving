# CARLA SAC + ROS2 Training Project

Deep Reinforcement Learning for autonomous driving using Soft Actor-Critic (SAC) with RLlib in CARLA simulator, integrated with ROS2.

## 🚗 Overview

This project implements a complete training pipeline for autonomous driving agents using:
- **SAC (Soft Actor-Critic)** with RLlib for reinforcement learning
- **CARLA Simulator** (v0.9.x) for realistic driving simulation
- **ROS2** for sensor data publishing and control
- **Custom Gym Environment** with LiDAR BEV perception

## 📋 Features

- ✅ Custom CARLA Gym environment with multi-modal observations
- ✅ LiDAR Bird's Eye View (BEV) processing
- ✅ SAC training with RLlib and Ray
- ✅ ROS2 integration for sensor publishing and control
- ✅ Comprehensive reward shaping for autonomous driving
- ✅ Tensorboard logging and visualization
- ✅ Model checkpointing and evaluation
- ✅ Configurable via YAML files

## 🛠️ Installation

### Prerequisites

- Ubuntu 20.04/22.04
- Python 3.10+
- CARLA Simulator 0.9.15+
- ROS2 (Humble/Iron/Jazzy)
- CUDA-capable GPU (recommended)

### 1. Clone Repository

```bash
cd ~/Desktop
# Project already created at: carla_sac_ros2_training
cd carla_sac_ros2_training
```

### 2. Setup Virtual Environment

```bash
# Create virtual environment
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Install package in development mode
pip install -e .
```

### 3. Install CARLA

Download and install CARLA 0.9.15:

```bash
# Download CARLA (example)
wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.15.tar.gz

# Extract
sudo mkdir -p /opt/carla-simulator
sudo tar -xzf CARLA_0.9.15.tar.gz -C /opt/carla-simulator

# Add CARLA Python API to path
export PYTHONPATH=$PYTHONPATH:/opt/carla-simulator/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg
```

### 4. Install ROS2

Follow official ROS2 installation guide: https://docs.ros.org/en/humble/Installation.html

```bash
# Source ROS2 (add to ~/.bashrc)
source /opt/ros/humble/setup.bash
```

## 🚀 Quick Start

### 1. Start CARLA Server

```bash
# In terminal 1
cd /opt/carla-simulator
./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2000
```

### 2. Start Training

```bash
# In terminal 2
cd ~/Desktop/carla_sac_ros2_training
source venv/bin/activate

# Start training
bash scripts/start_training.sh
# Or with custom config:
# bash scripts/start_training.sh config/sac_config.yaml
```

### 3. Monitor Training

```bash
# In terminal 3
tensorboard --logdir=data/tensorboard
# Open browser: http://localhost:6006
```

### 4. Evaluate Model

```bash
# After training
bash scripts/evaluate_model.sh data/checkpoints/best_model 10
```

## 📁 Project Structure

```
carla_sac_ros2_training/
├── src/
│   ├── carla_gym_env/       # CARLA Gym environment
│   ├── ros2_bridge/         # ROS2 integration
│   ├── sac_trainer/         # SAC training scripts
│   └── utils/               # Utilities
├── config/                  # Configuration files
├── launch/                  # ROS2 launch files
├── data/                    # Training data
│   ├── checkpoints/
│   ├── logs/
│   └── tensorboard/
├── tests/                   # Unit tests
├── scripts/                 # Helper scripts
├── requirements.txt
├── setup.py
└── README.md
```

## ⚙️ Configuration

### SAC Hyperparameters (`config/sac_config.yaml`)

```yaml
sac:
  gamma: 0.99
  lr: 0.0003
  train_batch_size: 256
  num_rollout_workers: 4
  num_gpus: 1
```

### CARLA Settings (`config/carla_config.yaml`)

```yaml
carla:
  host: localhost
  port: 2000
  map: Town01
  delta_seconds: 0.05
```

### Reward Function (`config/carla_config.yaml`)

```yaml
rewards:
  w_progress: 1.0
  w_comfort: 0.1
  w_collision: 200.0
  w_lane_deviation: 0.5
  w_speed: 0.2
```

## 🎯 Training Details

### State Space
- **LiDAR BEV**: 256×256×3 occupancy grid
- **Ego State**: [speed, heading, steering, acceleration, lateral_offset, heading_error]

### Action Space
- **Continuous**: [steering ∈ [-1,1], throttle ∈ [0,1], brake ∈ [0,1]]

### Reward Components
1. **Progress**: Forward movement aligned with road
2. **Comfort**: Penalty for jerky movements
3. **Collision**: Large negative reward
4. **Lane Deviation**: Penalty for off-center driving
5. **Speed Tracking**: Reward for maintaining target speed

## 📊 Monitoring & Visualization

### Tensorboard

```bash
tensorboard --logdir=data/tensorboard
```

### Training Logs

```bash
tail -f data/logs/sac_town01.log
```

### Plot Training Curves

```python
from src.utils.visualization import plot_training_curves

plot_training_curves('data/logs/training_metrics.csv', show=True)
```

## 🧪 Testing

Run unit tests:

```bash
pytest tests/ -v
```

Run specific test:

```bash
pytest tests/test_env.py -v
```

## 🔧 ROS2 Integration

### Launch ROS2 Nodes

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Launch CARLA ROS2 bridge
ros2 launch launch/carla_training.launch.py
```

### Published Topics

- `/carla/ego_vehicle/lidar/point_cloud2` - LiDAR data
- `/carla/ego_vehicle/camera/rgb/image_raw` - Camera images
- `/carla/ego_vehicle/imu` - IMU data
- `/carla/ego_vehicle/odometry` - Vehicle odometry
- `/carla/ego_vehicle/speedometer` - Vehicle speed

### Subscribed Topics

- `/carla/ego_vehicle/control_cmd` - Control commands

## 📈 Results

Training metrics are saved in:
- `data/logs/` - Text logs
- `data/tensorboard/` - Tensorboard logs
- `data/checkpoints/` - Model checkpoints

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## 📝 License

This project is licensed under the MIT License.

## 🙏 Acknowledgments

- [CARLA Simulator](https://carla.org/)
- [RLlib](https://docs.ray.io/en/latest/rllib/index.html)
- [ROS2](https://docs.ros.org/)

## 📧 Contact

For questions or issues, please open an issue on GitHub.

---

**Happy Training! 🚗💨**
