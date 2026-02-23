# Quick Start Guide

## Prerequisites

1. **CARLA Simulator 0.9.16** installed and running
2. **Python 3.10** installed
3. **CUDA** (optional but recommended for GPU acceleration)

## Step 1: Setup Environment

```bash
cd /home/a/Desktop/CARLA_0.9.16/carla_lstm_mpc_project

# Run setup script
./setup.sh

# Or manually:
# 1. Install dependencies
pip install -r requirements.txt

# 2. Add CARLA to PYTHONPATH
export PYTHONPATH=$PYTHONPATH:/home/a/Desktop/CARLA_0.9.16/PythonAPI/carla/dist/carla-0.9.16-py3.10-linux-x86_64.egg
```

## Step 2: Start CARLA Simulator

In a separate terminal:

```bash
cd /home/a/Desktop/CARLA_0.9.16
./CarlaUE4.sh
```

Wait for CARLA to fully load (you'll see the CARLA window).

## Step 3: Test Components (Optional)

Test individual components before running full system:

```bash
python test_components.py
```

This verifies:
- ResNet encoder works
- LSTM predictor works
- MPC controller works
- Visualization display works

## Step 4: Run the System

### Inference Mode (Autonomous Control)

```bash
python main.py --mode inference
```

The system will:
1. Connect to CARLA server
2. Load Town04
3. Spawn Tesla Model 3 at spawn point 0
4. Start autonomous driving with real-time visualization

**Note:** The LSTM model is not trained initially, so predictions will be random. However, the MPC controller will still work using simple forward prediction. For better performance, train the LSTM model first using collected data.

### Data Collection Mode

```bash
python main.py --mode collect
```

This collects:
- RGB camera images
- Vehicle states (x, y, yaw, velocity)
- Control actions (steering, throttle, brake)

Data is saved to `data/run_TIMESTAMP/`

## Controls

- **Close Window:** Click the X button or press Ctrl+C in terminal
- **Emergency Stop:** Press Ctrl+C in terminal

## Troubleshooting

### "Connection failed" error
- Make sure CARLA server is running
- Check that port 2000 is not blocked
- Verify CARLA Python API is in PYTHONPATH

### "Vehicle not found" error
- Ensure you're using CARLA 0.9.16
- Check that vehicle blueprint exists: `vehicle.tesla.model3`

### "MPC optimization failed" warnings
- This is normal occasionally
- System will use previous control as fallback
- Check vehicle state is valid

### Visualization window doesn't appear
- Check pygame is installed: `pip install pygame`
- Try running with `DISPLAY=:0 python main.py --mode inference`

## Next Steps

1. **Collect Training Data:** Run data collection mode to gather training data
2. **Train LSTM:** Use collected data to train the LSTM predictor
3. **Fine-tune MPC:** Adjust weights in `config.yaml` for better control
4. **Extend System:** Add obstacle detection, traffic, etc.

## File Structure

```
carla_lstm_mpc_project/
├── main.py              # Main entry point
├── config.yaml          # Configuration
├── requirements.txt     # Dependencies
├── setup.sh            # Setup script
├── test_components.py  # Component tests
│
├── carla_env/          # CARLA interface
├── perception/         # ResNet encoder
├── temporal/           # LSTM predictor
├── control/            # MPC controller
├── visualization/      # Real-time display
├── logs/               # Log files
└── data/               # Collected data
```

## Configuration

Edit `config.yaml` to customize:
- CARLA connection (host, port)
- Camera settings (resolution, FOV)
- Model parameters (feature dim, LSTM size)
- MPC parameters (horizon, weights)
- Visualization options

## Performance Tips

- Use GPU for faster ResNet encoding
- Reduce MPC horizon for faster optimization
- Lower camera resolution for better FPS
- Disable graphs in visualization for better performance

