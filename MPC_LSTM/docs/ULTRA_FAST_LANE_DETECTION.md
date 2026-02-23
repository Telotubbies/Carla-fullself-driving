# Ultra-Fast-Lane-Detection-v2 Integration Guide

## 📋 Overview

This project integrates [Ultra-Fast-Lane-Detection-v2](https://github.com/cfzd/Ultra-Fast-Lane-Detection-v2) pre-trained models for lane detection, providing an alternative to the custom U-Net model.

## 🎯 Features

- **Pre-trained Models**: Use state-of-the-art lane detection models trained on CULane, TuSimple, or CurveLanes datasets
- **High Performance**: F1 scores of 75-81% on CULane, 96%+ accuracy on TuSimple
- **Fast Inference**: Optimized for real-time lane detection
- **Easy Integration**: Drop-in replacement for U-Net lane detection

## 📥 Download Pre-trained Models

### Option 1: Using Download Script

```bash
# Download CULane ResNet18 model (recommended for CARLA)
./scripts/download_ultra_fast_model.sh culane res18 weights

# Download CULane ResNet34 model (better accuracy)
./scripts/download_ultra_fast_model.sh culane res34 weights

# Download TuSimple ResNet18 model
./scripts/download_ultra_fast_model.sh tusimple res18 weights
```

### Option 2: Manual Download

1. Visit the [Ultra-Fast-Lane-Detection-v2 repository](https://github.com/cfzd/Ultra-Fast-Lane-Detection-v2)
2. Download the desired pre-trained model from the "Trained models" section
3. Save to `weights/` directory with naming: `ufldv2_{dataset}_{backbone}.pth`

**Available Models:**

| Dataset    | Backbone | F1/Accuracy | Download Link |
|------------|----------|-------------|---------------|
| CULane     | ResNet18 | 75.0 F1     | [Google Drive](https://drive.google.com/file/d/1oEjJraFr-3lxhX_OXduAGFWalWa6Xh3W/view?usp=sharing) |
| CULane     | ResNet34 | 76.0 F1     | [Google Drive](https://drive.google.com/file/d/1AjnvAD3qmqt_dGPveZJsLZ1bOyWv62Yj/view?usp=sharing) |
| TuSimple   | ResNet18 | 96.11 Acc   | [Google Drive](https://drive.google.com/file/d/1Clnj9-dLz81S3wXiYtlkc4HVusCb978t/view?usp=sharing) |
| TuSimple   | ResNet34 | 96.24 Acc   | [Google Drive](https://drive.google.com/file/d/1pkz8homK433z39uStGK3ZWkDXrnBAMmX/view?usp=sharing) |
| CurveLanes | ResNet18 | 80.42 F1    | [Google Drive](https://drive.google.com/file/d/1VfbUvorKKMG4tUePNbLYPp63axgd-8BX/view?usp=sharing) |
| CurveLanes | ResNet34 | 81.34 F1    | [Google Drive](https://drive.google.com/file/d/1O1kPSr85Icl2JbwV3RBlxWZYhLEHo8EN/view?usp=sharing) |

## ⚙️ Configuration

Update `config.yaml` to use Ultra-Fast-Lane-Detection-v2:

```yaml
perception:
  lane_detection_model_path: "weights/ufldv2_culane_res18.pth"
  lane_detection_model_type: "ultra_fast"  # Change from "unet" to "ultra_fast"
  use_carla_lane_detection: false
```

## 💻 Usage

### In Python Code

```python
from perception.lane_detector import LaneDetector

# Initialize with Ultra-Fast-Lane-Detection-v2
detector = LaneDetector(
    model_path="weights/ufldv2_culane_res18.pth",
    model_type="ultra_fast"  # Specify model type
)

# Detect lanes
lane_mask, lane_features = detector.detect_lanes(image)
```

### Comparison: U-Net vs Ultra-Fast

```python
# U-Net (custom trained)
detector_unet = LaneDetector(
    model_path="data/autopilot_20260208_150902/lane_unet_model/lane_unet_final.pth",
    model_type="unet"  # or omit (default)
)

# Ultra-Fast-Lane-Detection-v2 (pre-trained)
detector_ultra = LaneDetector(
    model_path="weights/ufldv2_culane_res18.pth",
    model_type="ultra_fast"
)
```

## 🔧 Implementation Details

### Architecture

- **Module**: `perception/ultra_fast_lane_detector.py`
- **Wrapper Class**: `UltraFastLaneDetector`
- **Integration**: `LaneDetector` class supports both model types

### Output Format

Both models output:
- **lane_mask**: Binary lane mask (H, W) with values 0-255
- **lane_features**: Feature vector (128,) for downstream tasks

### Model Input

- **Ultra-Fast-Lane-Detection-v2**: Input size (320, 1600) for CULane
- **U-Net**: Input size (256, 256) - resized from original image

## 📊 Performance Comparison

| Model | Dataset | F1/Accuracy | Speed | Notes |
|-------|---------|-------------|-------|-------|
| U-Net (custom) | CARLA | ~70% (estimated) | Fast | Trained on CARLA data |
| Ultra-Fast Res18 | CULane | 75.0 F1 | Very Fast | Pre-trained, generalizes well |
| Ultra-Fast Res34 | CULane | 76.0 F1 | Fast | Better accuracy, slightly slower |

## 🚀 Recommendations

1. **For CARLA Simulation**: Use **CULane ResNet18** model
   - Best balance of speed and accuracy
   - Trained on similar road scenarios
   - F1 score: 75.0

2. **For Higher Accuracy**: Use **CULane ResNet34** model
   - Better accuracy (76.0 F1)
   - Slightly slower inference

3. **For Real-World Data**: Use **TuSimple ResNet34** model
   - Highest accuracy (96.24%)
   - Trained on highway scenarios

## ⚠️ Important Notes

1. **Model Architecture**: The current implementation uses a placeholder model structure. For production use, you need to:
   - Clone the [Ultra-Fast-Lane-Detection-v2 repository](https://github.com/cfzd/Ultra-Fast-Lane-Detection-v2)
   - Import the actual model architecture (`parsingNet` or similar)
   - Update `ultra_fast_lane_detector.py` to use the real model

2. **Output Conversion**: Ultra-Fast-Lane-Detection-v2 outputs lane points/segments, which need to be converted to binary masks. The current implementation includes a placeholder for this conversion.

3. **Dependencies**: Ensure you have the required dependencies:
   ```bash
   pip install torch torchvision
   ```

## 🔗 References

- [Ultra-Fast-Lane-Detection-v2 GitHub](https://github.com/cfzd/Ultra-Fast-Lane-Detection-v2)
- [Paper: Ultra Fast Deep Lane Detection With Hybrid Anchor Driven Ordinal Classification (TPAMI 2022)](https://arxiv.org/abs/2203.10324)

## 📝 TODO

- [ ] Integrate actual Ultra-Fast-Lane-Detection-v2 model architecture
- [ ] Implement proper output-to-mask conversion
- [ ] Add model validation and testing
- [ ] Benchmark performance vs U-Net
- [ ] Add support for different input sizes

