# CARLA LSTM-MPC Autonomous Driving Project

**Production-Ready Autonomous Driving Pipeline for CARLA Simulator**

ระบบขับรถอัตโนมัติใน CARLA Simulator โดยใช้ ResNet, LSTM และ MPC

> 📖 **For Production Documentation**: See [README_PRODUCTION.md](README_PRODUCTION.md)  
> 📁 **For Project Structure**: See [PROJECT_STRUCTURE_PRODUCTION.md](PROJECT_STRUCTURE_PRODUCTION.md)

## 📁 โครงสร้างโปรเจกต์

```
carla_lstm_mpc_project/
├── carla_env/              # CARLA environment & client
├── perception/             # ResNet feature encoder
├── temporal/               # LSTM predictor
├── control/                # MPC controller
├── visualization/          # Pygame GUI
├── training/               # Training scripts
│   ├── collect_autopilot_data.py
│   ├── collect_diverse_data.py
│   ├── data_augmentation.py
│   ├── data_preprocessing.py
│   ├── extract_features.py
│   └── train_lstm.py
├── utils/                  # Utilities
├── scripts/                # Helper scripts
│   ├── setup/              # Setup & installation
│   ├── monitoring/         # Check & monitor
│   └── data_collection/    # Data collection
├── docs/                   # Documentation
├── data/                   # Training data
├── logs/                   # Log files
├── config.yaml             # Configuration
├── main.py                 # Main entry point
└── requirements.txt        # Dependencies
```

## 🚀 Quick Start

### 1. Setup
```bash
# Install dependencies
pip install -r requirements.txt

# Setup ROCm (for AMD GPU)
./scripts/setup/install_rocm.sh
```

### 2. Data Collection
```bash
# Start CARLA + collect data
./scripts/data_collection/start_all.sh

# Or manually
python3 training/collect_autopilot_data.py --frames 20000
```

### 3. Training
```bash
# Full pipeline (collect → preprocess → extract → train)
./scripts/data_collection/run_full_pipeline.sh
```

### 4. Inference
```bash
# Run autonomous driving
python3 main.py --mode inference
```

## 📊 Monitoring

```bash
# Check system status
./scripts/monitoring/check_all.sh

# Monitor data collection
./scripts/monitoring/check_pipeline.sh

# Check ResNet & data quality
python3 scripts/monitoring/check_resnet_and_data.py
```

## 📚 Documentation

- [Data Collection Guide](docs/DATA_COLLECTION_GUIDE.md)
- [Quick Start (Thai)](docs/QUICKSTART_TH.md)

## 🔧 Configuration

แก้ไข `config.yaml` เพื่อปรับแต่ง:
- CARLA settings (town, vehicle, weather)
- Model parameters (ResNet, LSTM, MPC)
- Data collection settings
- Visualization settings

## 📝 Scripts Overview

### Setup Scripts (`scripts/setup/`)
- `setup.sh` - Initial project setup
- `install_rocm.sh` - Install ROCm for AMD GPU

### Monitoring Scripts (`scripts/monitoring/`)
- `check_all.sh` - Comprehensive system check
- `check_pipeline.sh` - Data collection status
- `check_resnet_and_data.py` - Data quality check

### Data Collection Scripts (`scripts/data_collection/`)
- `start_all.sh` - Start CARLA + data collection
- `collect_diverse_pipeline.sh` - Diverse data collection
- `run_full_pipeline.sh` - Complete training pipeline

## 🎯 Features

- ✅ ResNet18 feature extraction
- ✅ LSTM temporal prediction
- ✅ MPC control
- ✅ Real-time visualization
- ✅ Diverse data collection
- ✅ Data augmentation
- ✅ ROCm support (AMD GPU)

## 📄 License

MIT License
