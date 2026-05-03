# Getting Started with CARLA SAC Training

## Quick Setup Guide

### 1. Install Dependencies

```bash
cd /home/supawich/Desktop/carla_sac_ros2_training

# Activate virtual environment
source venv/bin/activate

# Install Python packages
pip install -r requirements.txt

# Install in development mode
pip install -e .
```

### 2. Setup CARLA

Make sure CARLA is installed and the Python API is accessible:

```bash
# Set CARLA path (adjust to your installation)
export CARLA_PATH=/opt/carla-simulator

# Add to PYTHONPATH
export PYTHONPATH=$PYTHONPATH:$CARLA_PATH/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg

# Test CARLA import
python -c "import carla; print('CARLA imported successfully')"
```

### 3. Setup ROS2 (Optional)

If using ROS2 integration:

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Verify ROS2
ros2 --version
```

## Running Your First Training

### Step 1: Start CARLA Server

```bash
# Terminal 1
cd /opt/carla-simulator
./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2000
```

### Step 2: Start Training

```bash
# Terminal 2
cd /home/supawich/Desktop/carla_sac_ros2_training
source venv/bin/activate

# Option 1: Use helper script
bash scripts/start_training.sh

# Option 2: Direct Python
python src/sac_trainer/train.py --config config/sac_config.yaml
```

### Step 3: Monitor Progress

```bash
# Terminal 3 - Tensorboard
tensorboard --logdir=data/tensorboard

# Terminal 4 - Watch logs
tail -f data/logs/sac_town01.log
```

## Configuration Tips

### Adjust Training Speed

Edit `config/sac_config.yaml`:

```yaml
sac:
  num_rollout_workers: 4  # Increase for faster data collection
  num_gpus: 1             # Set to 0 if no GPU
  train_batch_size: 256   # Decrease if running out of memory
```

### Change CARLA Map

Edit `config/carla_config.yaml`:

```yaml
carla:
  map: Town01  # Options: Town01-Town07, Town10HD
```

### Modify Reward Function

Edit `config/carla_config.yaml`:

```yaml
rewards:
  w_progress: 1.0        # Increase for faster driving
  w_collision: 200.0     # Increase to penalize collisions more
  w_lane_deviation: 0.5  # Increase for better lane keeping
```

## Evaluation

After training, evaluate your model:

```bash
# Evaluate best model
bash scripts/evaluate_model.sh data/checkpoints/best_model 10

# Or with Python
python src/sac_trainer/evaluation.py data/checkpoints/best_model --episodes 10
```

## Troubleshooting

### CARLA Connection Issues

```bash
# Check if CARLA is running
python -c "
import carla
client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
print(client.get_server_version())
"
```

### GPU Memory Issues

Reduce batch size or number of workers:

```yaml
sac:
  train_batch_size: 128  # Reduce from 256
  num_rollout_workers: 2  # Reduce from 4
```

### Import Errors

Make sure virtual environment is activated and all dependencies are installed:

```bash
source venv/bin/activate
pip install -r requirements.txt
```

## Next Steps

1. **Experiment with hyperparameters** in `config/sac_config.yaml`
2. **Try different maps** in CARLA
3. **Modify reward function** for specific behaviors
4. **Add traffic** by editing `config/carla_config.yaml`
5. **Visualize results** using Tensorboard and plotting utilities

## Useful Commands

```bash
# List all checkpoints
ls -lh data/checkpoints/

# View training metrics
cat data/logs/training_metrics.csv

# Plot training curves
python -c "from src.utils.visualization import plot_training_curves; plot_training_curves('data/logs/training_metrics.csv', show=True)"

# Run tests
pytest tests/ -v
```

## Support

For issues or questions, check:
- README.md for detailed documentation
- Configuration files in `config/`
- Test files in `tests/` for examples

Happy training! 🚗💨
