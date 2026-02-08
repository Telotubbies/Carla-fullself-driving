# 🔄 Automated Flow Diagram

## Complete Pipeline Flow

```
┌─────────────────────────────────────────────────────────────┐
│                    🚀 AUTO FLOW START                       │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  🔍 STEP 0: Check Prerequisites                             │
│     • Python3                                               │
│     • CARLA running                                         │
│     • Dependencies (PyTorch, CasADi)                        │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  📊 STEP 1: Data Collection                                │
│     • Collect 50,000 frames from CARLA autopilot           │
│     • Save: images/, data.csv                              │
│     • Skip if already exists (≥10k frames)                 │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  🛣️  STEP 2: Create Lane Labels                           │
│     • Generate lane masks from CARLA map API               │
│     • Fallback: Image processing (Canny edge)              │
│     • Save: lane_masks/*.png                               │
│     • Skip if already exists (≥1k masks)                   │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  🎯 STEP 3: Fine-tune ResNet for Lane Detection            │
│     • Train ResNet with lane masks                          │
│     • 300 epochs, batch size 16                            │
│     • Save: resnet_lane_model/resnet_lane_final.pth        │
│     • Skip if model already exists                         │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  🔍 STEP 4: Extract Features                               │
│     • Use fine-tuned ResNet to extract features            │
│     • Process all images → feature vectors                 │
│     • Save: features.npy                                   │
│     • Skip if already exists                                │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  🧠 STEP 5: Train LSTM                                     │
│     • Train LSTM with feature sequences                    │
│     • 150 epochs, batch size 64                            │
│     • Advanced: Attention, Combined Loss, LR scheduling    │
│     • Save: lstm_model/best_model.pth                      │
│     • Skip if model already exists                         │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  ⚙️  STEP 6: Update Configuration                          │
│     • Update config.yaml with model paths                  │
│     • Set ResNet and LSTM model paths                      │
│     • Enable fine-tuned models                             │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  🚗 STEP 7: Run Inference with GUI                        │
│     • Start CARLA if needed                                │
│     • Load trained models                                   │
│     • Run autonomous driving                                │
│     • Real-time visualization (Pygame)                     │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                    ✅ FLOW COMPLETE                         │
└─────────────────────────────────────────────────────────────┘
```

## Usage

### Full Automated Flow
```bash
./scripts/entry_points/auto_flow.sh
```

### Skip Data Collection
```bash
./scripts/entry_points/auto_flow.sh --skip-data
```

### Skip Training (Inference Only)
```bash
./scripts/entry_points/auto_flow.sh --skip-training
```

### Inference Only
```bash
./scripts/entry_points/auto_flow.sh --inference-only
```

## Flow Features

### ✅ Smart Skipping
- Automatically skips steps if outputs already exist
- Checks file counts and model existence
- Continues from last incomplete step

### ✅ Error Handling
- Stops on error with clear messages
- Logs all steps to file
- CARLA auto-start/restart

### ✅ Progress Tracking
- Color-coded output
- Step-by-step logging
- Timestamp tracking

### ✅ Flexible Execution
- Full pipeline
- Skip data collection
- Skip training
- Inference only

## Log Files

All flow execution is logged to:
```
logs/auto_flow_YYYYMMDD_HHMMSS.log
```

## Cleanup

Clean old MPC files:
```bash
./scripts/maintenance/cleanup_mpc.sh
```

This removes:
- Python cache files (`__pycache__/`, `*.pyc`)
- Backup files (`*.bak`, `*.old`, `*~`)
- Unused files in `control/unused/` (keeps README.md)

