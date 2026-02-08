# Production-Level Project Structure

## 📁 Directory Organization

```
carla_lstm_mpc_project/
├── 📚 docs/                          # Documentation
│   ├── guides/                       # User guides and tutorials
│   ├── api/                          # API documentation
│   ├── architecture/                  # System architecture docs
│   ├── deployment/                    # Deployment guides
│   └── status/                       # Status reports
│
├── 📦 scripts/                        # All scripts organized by purpose
│   ├── entry_points/                  # Main entry point scripts
│   │   ├── run_complete_pipeline.sh
│   │   ├── run_full_auto_pipeline.sh
│   │   └── run_mpc_inference.sh
│   ├── maintenance/                   # Maintenance and organization
│   ├── setup/                         # Setup and installation
│   ├── monitoring/                    # Monitoring and checking
│   ├── data_collection/               # Data collection scripts
│   └── training/                      # Training scripts
│
├── ⚙️  configs/                        # Configuration files
│   ├── production/                    # Production configs
│   ├── development/                   # Development configs
│   └── testing/                        # Testing configs
│
├── 🧪 tests/                          # Test suite
│   ├── unit/                          # Unit tests
│   ├── integration/                   # Integration tests
│   └── e2e/                           # End-to-end tests
│
├── 📋 logs/                           # Log files
│   ├── training/                      # Training logs
│   ├── inference/                     # Inference logs
│   ├── errors/                        # Error logs
│   └── archived/                      # Archived logs
│
├── 🚀 deployment/                     # Deployment configurations
│   ├── docker/                        # Docker files
│   ├── kubernetes/                    # K8s manifests
│   └── scripts/                       # Deployment scripts
│
├── 🧠 Core Modules                    # Main Python modules
│   ├── carla_env/                     # CARLA environment
│   ├── perception/                    # Perception (ResNet)
│   ├── temporal/                      # Temporal (LSTM)
│   ├── control/                       # Control (MPC)
│   ├── visualization/                 # Visualization
│   ├── training/                      # Training scripts
│   └── utils/                         # Utilities
│
├── 📊 data/                           # Training data
├── main.py                            # Main entry point
├── config.yaml                        # Main config (symlink to configs/)
├── requirements.txt                   # Python dependencies
└── README.md                          # Main documentation
```

## 🎯 Entry Points

### Quick Start
```bash
# Run complete pipeline
./run_pipeline.sh

# Run inference only
./run_inference.sh

# Check system status
python3 scripts/view_status.py
```

### Direct Python Entry
```bash
# Main system
python3 main.py --mode inference
python3 main.py --mode collect

# Training
python3 training/train_lstm.py --data-dir data/autopilot_XXX
python3 training/finetune_resnet_lane.py --data-dir data/autopilot_XXX
```

## 📚 Documentation Structure

- **guides/**: User-facing guides (Quick Start, Tutorials)
- **api/**: API documentation for modules
- **architecture/**: System design and architecture
- **deployment/**: Deployment instructions
- **status/**: Status reports and summaries

## 🔧 Configuration Management

- **configs/development/**: Development configuration
- **configs/production/**: Production configuration
- **configs/testing/**: Testing configuration
- **config.yaml**: Symlink to active config

## 🧪 Testing Structure

- **unit/**: Unit tests for individual components
- **integration/**: Integration tests for module interactions
- **e2e/**: End-to-end tests for full pipeline

## 📋 Logging Structure

- **training/**: Training-related logs
- **inference/**: Inference and runtime logs
- **errors/**: Error logs
- **archived/**: Archived logs

## 🚀 Deployment

- **docker/**: Docker configurations
- **kubernetes/**: Kubernetes manifests
- **scripts/**: Deployment automation scripts

