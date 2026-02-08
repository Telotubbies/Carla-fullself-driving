# Project Structure

โครงสร้างโปรเจกต์ที่จัดระเบียบแล้ว

## 📁 Directory Structure

```
carla_lstm_mpc_project/
│
├── 📂 carla_env/              # CARLA environment & client
│   ├── carla_client.py
│   └── sensors.py
│
├── 📂 perception/             # ResNet feature encoder
│   └── resnet_encoder.py
│
├── 📂 temporal/               # LSTM predictor
│   └── lstm_predictor.py
│
├── 📂 control/                # MPC controller
│   └── mpc_controller.py
│
├── 📂 visualization/          # Pygame GUI
│   └── display.py
│
├── 📂 training/               # Training scripts
│   ├── collect_autopilot_data.py
│   ├── collect_diverse_data.py
│   ├── data_augmentation.py
│   ├── data_preprocessing.py
│   ├── extract_features.py
│   └── train_lstm.py
│
├── 📂 utils/                  # Utilities
│   ├── device_utils.py
│   └── database.py
│
├── 📂 scripts/                # Helper scripts
│   ├── setup/                 # Setup & installation
│   │   ├── setup.sh
│   │   ├── install_rocm.sh
│   │   └── fix_*.sh
│   │
│   ├── monitoring/            # Check & monitor
│   │   ├── check_all.sh
│   │   ├── check_pipeline.sh
│   │   ├── check_resnet_and_data.py
│   │   └── monitor_*.sh
│   │
│   └── data_collection/       # Data collection
│       ├── start_all.sh
│       ├── collect_diverse_pipeline.sh
│       ├── run_full_pipeline.sh
│       └── auto_complete_*.sh
│
├── 📂 docs/                   # Documentation
│   ├── DATA_COLLECTION_GUIDE.md
│   ├── QUICKSTART_TH.md
│   ├── README_*.md
│   └── *STATUS*.md
│
├── 📂 data/                   # Training data
│   └── autopilot_*/
│
├── 📂 logs/                   # Log files
│   ├── main.log
│   └── old/                   # Old logs
│
├── 📂 tests/                  # Test files
│   └── test_components.py
│
├── 📄 main.py                 # Main entry point
├── 📄 config.yaml             # Configuration
├── 📄 requirements.txt        # Dependencies
└── 📄 README.md               # Main README
```

## 🔗 Symlinks

Scripts ที่ใช้บ่อยมี symlink ที่ root:
- `check_all.sh` → `scripts/monitoring/check_all.sh`
- `start_all.sh` → `scripts/data_collection/start_all.sh`

## 📝 File Categories

### Core Code
- `main.py` - Main entry point
- `carla_env/` - CARLA interface
- `perception/` - Vision processing
- `temporal/` - LSTM prediction
- `control/` - MPC control
- `visualization/` - GUI

### Training
- `training/` - All training scripts
- `data/` - Training datasets
- `logs/` - Training logs

### Scripts
- `scripts/setup/` - Installation & setup
- `scripts/monitoring/` - Status checks
- `scripts/data_collection/` - Data collection automation

### Documentation
- `docs/` - All documentation files
- `README.md` - Main documentation

## 🎯 Quick Access

```bash
# Check system
./check_all.sh

# Start data collection
./start_all.sh

# View docs
ls docs/
```

