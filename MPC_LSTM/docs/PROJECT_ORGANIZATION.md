# 📁 Project Organization Summary

## Overview

This document describes the reorganized project structure with a centralized orchestrator system and modular Python files.

## Key Improvements

### 1. Central Orchestrator System

**Location**: `core/orchestrator.py`, `scripts/orchestrator_main.py`

**Features**:
- Phase-based workflow execution
- Dependency management
- State persistence (resume from checkpoint)
- Error recovery
- Progress tracking

**Usage**:
```bash
# Run all phases
python3 scripts/orchestrator_main.py

# Run specific phases
python3 scripts/orchestrator_main.py --phases collect_data,finetune_resnet

# Resume from checkpoint
python3 scripts/orchestrator_main.py --resume

# Check status
python3 scripts/orchestrator_main.py --status
```

### 2. Modular System Architecture

**Before**: `main.py` (770 lines) - monolithic

**After**: Split into modules:
- `core/system_base.py` - Base functionality (validation, logging)
- `core/system_inference.py` - Inference control loop
- `core/system_collection.py` - Data collection loop
- `main.py` - Main entry point (simplified)

### 3. Phase Definitions

**Location**: `core/phases.py`

**Phases**:
1. `collect_data` - Collect training data
2. `create_lane_labels` - Create lane masks
3. `finetune_resnet` - Fine-tune ResNet
4. `extract_features` - Extract features
5. `train_lstm` - Train LSTM
6. `update_config` - Update config.yaml
7. `run_inference` - Run inference

### 4. File Organization

**Large Files Identified**:
- `main.py` (770 lines) → Split into modules ✅
- `collect_diverse_data.py` (572 lines) → To be refactored
- `display.py` (563 lines) → To be refactored
- `train_lstm.py` (524 lines) → To be refactored

## Directory Structure

```
carla_lstm_mpc_project/
├── core/                          # Core system modules
│   ├── orchestrator.py           # Pipeline orchestrator
│   ├── phases.py                 # Phase definitions
│   ├── system_base.py            # Base system functionality
│   ├── system_inference.py       # Inference system
│   ├── system_collection.py      # Data collection system
│   ├── config.py                 # Config manager
│   ├── factories.py              # Factory patterns
│   ├── interfaces.py            # Interfaces
│   ├── validators.py             # Validators
│   └── exceptions.py             # Custom exceptions
│
├── scripts/
│   ├── orchestrator_main.py     # Orchestrator entry point
│   ├── run_orchestrator.sh       # Shell wrapper
│   ├── entry_points/             # Other entry points
│   ├── training/                 # Training scripts
│   ├── monitoring/               # Monitoring scripts
│   └── ...
│
├── training/                     # Training scripts
│   ├── collect_autopilot_data.py
│   ├── finetune_resnet_lane.py
│   ├── train_lstm.py
│   └── ...
│
└── docs/
    ├── ORCHESTRATOR_GUIDE.md     # Orchestrator documentation
    └── PROJECT_ORGANIZATION.md   # This file
```

## Workflow

### Traditional Approach (Before)
```bash
# Manual step-by-step
python3 training/collect_autopilot_data.py
python3 training/create_lane_labels.py
python3 training/finetune_resnet_lane.py
python3 training/extract_features.py
python3 training/train_lstm.py
python3 main.py --mode inference
```

### Orchestrator Approach (After)
```bash
# Single command - automatic
python3 scripts/orchestrator_main.py

# Or with options
python3 scripts/orchestrator_main.py --resume
python3 scripts/orchestrator_main.py --phases train_lstm,run_inference
```

## Benefits

1. **Centralized Control**: Single entry point for entire pipeline
2. **Resumability**: Continue from where you left off
3. **Visibility**: Clear status of each phase
4. **Flexibility**: Run specific phases or all
5. **Maintainability**: Easy to add/modify phases
6. **Modularity**: Smaller, focused files

## Next Steps

1. ✅ Create orchestrator system
2. ✅ Split main.py into modules
3. ⏳ Refactor `collect_diverse_data.py` (572 lines)
4. ⏳ Refactor `display.py` (563 lines)
5. ⏳ Refactor `train_lstm.py` (524 lines)
6. ⏳ Update main.py to use new modules
7. ⏳ Add unit tests for orchestrator

## Migration Guide

### For Existing Scripts

No changes needed! The orchestrator uses existing scripts:
- `training/collect_autopilot_data.py`
- `training/create_lane_labels.py`
- `training/finetune_resnet_lane.py`
- `training/extract_features.py`
- `training/train_lstm.py`

### For New Phases

1. Add phase function to `core/phases.py`
2. Register phase in `scripts/orchestrator_main.py`
3. Define dependencies
4. Test phase execution

## State Management

State is persisted in `logs/pipeline_state.json`:

```json
{
  "phases": {
    "collect_data": {
      "status": "completed",
      "start_time": 1234567890.0,
      "end_time": 1234567900.0
    }
  },
  "current_phase": "finetune_resnet",
  "last_updated": "2026-02-08T19:00:00"
}
```

## Error Handling

- **Stop on Error** (default): Pipeline stops at first error
- **Resume**: Continue from last completed phase
- **Reset**: Start fresh (clear state)

## Documentation

- `docs/ORCHESTRATOR_GUIDE.md` - Detailed orchestrator guide
- `docs/PROJECT_ORGANIZATION.md` - This file

