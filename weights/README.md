# Model Weights (LSTM / MPC)

## Required for inference

- **LSTM**: `data/autopilot_*/lstm_model/best_model.pth` (~5–6 MB)  
  - Set `config.yaml` → `temporal.trained_model_path` to this path.
- **ResNet lane** (optional): `data/autopilot_*/resnet_lane_model/resnet_lane_best.pth` (~43 MB)  
- **Ultra-Fast Lane** (optional): `weights/ufldv2_tusimple_res18.pth` or from [UFLDv2](https://github.com/cfzd/Ultra-Fast-Lane-Detection-v2) (~368 MB)

## Why weights are not in this repo

- GitHub file size limit is 100 MB. ResNet/UltraFast checkpoints are larger.
- LSTM `best_model.pth` is small enough but is binary; use one of the options below.

## Setup options

1. **Copy from your machine** (after training):  
   - Copy `data/autopilot_YYYYMMDD_HHMMSS/lstm_model/best_model.pth` into the same path in the cloned repo.
2. **Git LFS** (if you enable it):  
   - `git lfs track "*.pth"` then add and push `best_model.pth`.
3. **Release asset**:  
   - Upload `best_model.pth` (and optionally ResNet best) as a GitHub Release asset and document the download URL in this README.

## Config example

```yaml
temporal:
  trained_model_path: data/autopilot_20260208_150902/lstm_model/best_model.pth
```
