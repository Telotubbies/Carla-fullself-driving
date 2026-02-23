# 🎯 Pipeline Orchestrator Guide

## Overview

The Pipeline Orchestrator is a centralized system for managing the complete CARLA LSTM-MPC workflow in phases. It provides:

- **Phase-based execution**: Workflow divided into clear phases
- **Dependency management**: Automatic ordering based on dependencies
- **State persistence**: Resume from last checkpoint
- **Error recovery**: Stop on error or continue
- **Progress tracking**: Real-time status monitoring

## Architecture

```
orchestrator_main.py (Entry Point)
    ↓
PipelineOrchestrator (Core Engine)
    ↓
Phase Functions (from phases.py)
    ↓
Training/Inference Scripts
```

## Phases

### Phase 1: `collect_data`
- **Description**: Collect training data from CARLA autopilot
- **Dependencies**: None
- **Output**: `data/autopilot_TIMESTAMP/`

### Phase 2: `create_lane_labels`
- **Description**: Create lane masks from CARLA map
- **Dependencies**: `collect_data`
- **Output**: `data/autopilot_TIMESTAMP/lane_masks/`

### Phase 3: `finetune_resnet`
- **Description**: Fine-tune ResNet for lane detection
- **Dependencies**: `create_lane_labels`
- **Output**: `data/autopilot_TIMESTAMP/resnet_lane_model/`

### Phase 4: `extract_features`
- **Description**: Extract features using ResNet
- **Dependencies**: `finetune_resnet`
- **Output**: `data/autopilot_TIMESTAMP/features.npy`

### Phase 5: `train_lstm`
- **Description**: Train LSTM model
- **Dependencies**: `extract_features`
- **Output**: `data/autopilot_TIMESTAMP/lstm_model/`

### Phase 6: `update_config`
- **Description**: Update config.yaml with trained models
- **Dependencies**: `train_lstm`, `finetune_resnet`
- **Output**: Updated `config.yaml`

### Phase 7: `run_inference`
- **Description**: Run inference with trained models
- **Dependencies**: `update_config`
- **Output**: Real-time autonomous driving

## Usage

### Run All Phases

```bash
python3 scripts/orchestrator_main.py
```

### Run Specific Phases

```bash
python3 scripts/orchestrator_main.py --phases collect_data,finetune_resnet
```

### Resume from Last Checkpoint

```bash
python3 scripts/orchestrator_main.py --resume
```

### Reset All Phases

```bash
python3 scripts/orchestrator_main.py --reset
```

### Check Status

```bash
python3 scripts/orchestrator_main.py --status
```

## State Management

State is persisted in `logs/pipeline_state.json`:

```json
{
  "phases": {
    "collect_data": {
      "status": "completed",
      "start_time": 1234567890.0,
      "end_time": 1234567900.0,
      "duration": 10.0,
      "error": null
    }
  },
  "current_phase": "finetune_resnet",
  "last_updated": "2026-02-08T19:00:00"
}
```

## Error Handling

- **Stop on Error** (default): Pipeline stops at first error
- **Continue on Error**: Use `--no-stop-on-error` (not implemented yet)

## Integration with Existing Scripts

The orchestrator uses existing training scripts:
- `training/collect_autopilot_data.py`
- `training/create_lane_labels.py`
- `training/finetune_resnet_lane.py`
- `training/extract_features.py`
- `training/train_lstm.py`
- `scripts/entry_points/run_inference_refactored.py`

## Benefits

1. **Centralized Control**: Single entry point for entire pipeline
2. **Resumability**: Continue from where you left off
3. **Visibility**: Clear status of each phase
4. **Flexibility**: Run specific phases or all
5. **Maintainability**: Easy to add/modify phases

## Future Enhancements

- Parallel phase execution (where dependencies allow)
- Phase retry logic
- Email/notification on completion
- Web dashboard for monitoring
- Phase-specific configuration

