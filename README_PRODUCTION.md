# CARLA LSTM-MPC Autonomous Driving System

**Production-Ready Autonomous Driving Pipeline for CARLA Simulator**

[![Python 3.10](https://img.shields.io/badge/python-3.10-blue.svg)](https://www.python.org/downloads/)
[![CARLA 0.9.16](https://img.shields.io/badge/CARLA-0.9.16-orange.svg)](https://carla.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## 🎯 Overview

A complete end-to-end autonomous driving research pipeline using:
- **ResNet18** for visual feature extraction
- **LSTM** for temporal state prediction
- **MPC (Model Predictive Control)** for vehicle control
- **Real-time visualization** with Pygame

## 🏗️ Architecture

```
Camera → ResNet → LSTM → MPC → Vehicle Control
         ↓         ↓      ↓
      Features  States  Control
```

### Components

1. **Perception** (`perception/`)
   - ResNet18 encoder for feature extraction
   - Lane detection with fine-tuned ResNet
   - 512-dimensional feature vectors

2. **Temporal** (`temporal/`)
   - LSTM network for state prediction
   - Sequence buffer (10 frames)
   - Predicts: x, y, yaw, velocity

3. **Control** (`control/`)
   - MPC controller using CasADi
   - Kinematic bicycle model
   - Optimizes: steering, throttle, brake

4. **Visualization** (`visualization/`)
   - Real-time Pygame GUI
   - Camera view with overlays
   - Live graphs (speed, steering)

## 🚀 Quick Start

### Prerequisites

- Python 3.10+
- CARLA 0.9.16
- CUDA/ROCm (for GPU acceleration)
- Ubuntu 20.04+

### Installation

```bash
# Clone repository
git clone <repository-url>
cd carla_lstm_mpc_project

# Install dependencies
pip install -r requirements.txt

# Setup ROCm (for AMD GPU)
./scripts/setup/install_rocm.sh

# Run setup
./scripts/setup/setup.sh
```

### Run Complete Pipeline

```bash
# Automated pipeline (recommended)
./run_pipeline.sh

# This will:
# 1. Collect data (20k frames)
# 2. Create lane labels
# 3. Fine-tune ResNet for lane detection
# 4. Extract features
# 5. Train LSTM
# 6. Run inference with GUI
```

### Run Individual Steps

```bash
# 1. Data Collection
python3 training/collect_autopilot_data.py --frames 20000

# 2. Create Lane Labels
python3 training/create_lane_labels.py --images-dir data/autopilot_XXX/images

# 3. Fine-tune ResNet
python3 training/finetune_resnet_lane.py --data-dir data/autopilot_XXX --epochs 300

# 4. Extract Features
python3 training/extract_features.py --data-dir data/autopilot_XXX

# 5. Train LSTM
python3 training/train_lstm.py --data-dir data/autopilot_XXX --epochs 150

# 6. Run Inference
python3 main.py --mode inference
```

## 📊 Monitoring

### System Status

```bash
# View current status
python3 scripts/view_status.py

# View latest status log
./scripts/view_status_latest.sh

# Comprehensive check
./scripts/monitoring/check_all.sh
```

### Training Monitoring

```bash
# Monitor training progress
./scripts/monitoring/monitor_training.sh

# Check data quality
python3 scripts/monitoring/check_resnet_and_data.py
```

## 📁 Project Structure

```
carla_lstm_mpc_project/
├── carla_env/          # CARLA environment & client
├── perception/         # ResNet feature encoder
├── temporal/           # LSTM predictor
├── control/            # MPC controller
├── visualization/      # Pygame GUI
├── training/           # Training scripts
├── utils/              # Utilities
├── scripts/            # Helper scripts
├── docs/               # Documentation
├── configs/            # Configuration files
├── tests/              # Test suite
└── logs/               # Log files
```

See [PROJECT_STRUCTURE_PRODUCTION.md](PROJECT_STRUCTURE_PRODUCTION.md) for detailed structure.

## ⚙️ Configuration

Edit `config.yaml` to customize:

- **CARLA Settings**: Town, vehicle, weather, spawn point
- **Model Parameters**: ResNet, LSTM, MPC settings
- **Data Collection**: Frame count, collection strategy
- **Visualization**: Display settings, graph options

## 🧪 Testing

```bash
# Run all tests
pytest tests/

# Run unit tests
pytest tests/unit/

# Run integration tests
pytest tests/integration/

# Run E2E tests
pytest tests/e2e/
```

## 📚 Documentation

- [Quick Start Guide](docs/guides/QUICK_START.md)
- [Training Guide](docs/TRAINING_GUIDE.md)
- [Lane Detection Guide](docs/LANE_DETECTION_GUIDE.md)
- [Status Monitoring](docs/STATUS_MONITORING.md)
- [Project Structure](PROJECT_STRUCTURE_PRODUCTION.md)

## 🔧 Advanced Features

### ROCm Support (AMD GPU)

```bash
# Setup ROCm
./scripts/setup/install_rocm.sh

# Set environment variable
export HSA_OVERRIDE_GFX_VERSION=11.0.0
```

### Data Augmentation

```bash
# Apply data augmentation
python3 training/data_augmentation.py --data-dir data/autopilot_XXX
```

### Diverse Data Collection

```bash
# Collect diverse data
./scripts/data_collection/collect_diverse_pipeline.sh
```

## 🐛 Troubleshooting

### CARLA Connection Issues

```bash
# Check CARLA status
./scripts/monitoring/check_all.sh

# Restart CARLA
./scripts/data_collection/start_all.sh
```

### GPU Issues

```bash
# Check GPU
python3 -c "from utils.device_utils import get_device_info; print(get_device_info())"

# Fix ROCm
./scripts/setup/fix_rocm.sh
```

## 📈 Performance

- **Data Collection**: ~20k frames in 15-20 minutes
- **ResNet Fine-tuning**: ~300 epochs in 2-3 hours (GPU)
- **LSTM Training**: ~150 epochs in 1-2 hours (GPU)
- **Inference**: Real-time at 20 Hz

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests
5. Submit a pull request

## 📄 License

MIT License - see LICENSE file for details

## 🙏 Acknowledgments

- CARLA Simulator team
- PyTorch community
- CasADi optimization framework

## 📧 Contact

For questions or issues, please open an issue on GitHub.

---

**Status**: Production Ready ✅  
**Last Updated**: 2026-02-08

