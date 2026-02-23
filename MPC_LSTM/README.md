# MPC-LSTM Controller for CARLA Autonomous Driving

> **Model Predictive Control (MPC) with LSTM-based Prediction** for trajectory planning and vehicle control in the CARLA simulator.

---

## Overview

This module implements a **hybrid MPC-LSTM controller** that combines:

- **LSTM (Long Short-Term Memory)** — learns vehicle dynamics and predicts future states from sequential sensor data
- **MPC (Model Predictive Control)** — uses the LSTM prediction model as a forward dynamics model to compute optimal control inputs over a receding horizon

The controller is designed to integrate with the existing SAC-based pipeline in this repository and provides an alternative/complementary control strategy for:

- Smooth trajectory tracking
- Constraint-aware planning (lane boundaries, speed limits)
- Interpretable and predictable control behavior

---

## Architecture

```
Sensor Data (t-N ... t)
       │
       ▼
┌─────────────┐
│  LSTM Model │  ← Sequence encoder (hidden: 256, layers: 2)
│  (Predictor)│    Input: [speed, steering, GPS, waypoints]
└──────┬──────┘
       │  Predicted future states ŷ(t+1 ... t+H)
       ▼
┌─────────────┐
│  MPC Solver │  ← Receding horizon optimizer
│             │    Horizon H = 10 steps
│  minimize   │    dt = 0.1s
│  Σ cost(ŷ,u)│
└──────┬──────┘
       │  Optimal control u*(t)
       ▼
  [steering, throttle, brake]
```

### Key Components

| Component | Description | File |
|---|---|---|
| `LSTMDynamicsModel` | Predicts next N states from history | `lstm_model.py` |
| `MPCController` | Receding horizon optimizer | `mpc_controller.py` |
| `MPCLSTMAgent` | Combined agent interface | `mpc_lstm_agent.py` |
| `DataCollector` | Collects rollout data for training | `data_collector.py` |
| `ModelTrainer` | Trains LSTM on collected data | `trainer.py` |

---

## LSTM Dynamics Model

### Input Features (per timestep)

```python
state = [
    velocity_x,      # longitudinal speed  (m/s)
    velocity_y,      # lateral speed       (m/s)
    yaw_rate,        # angular velocity    (rad/s)
    steering,        # steering angle      [-1, 1]
    throttle,        # throttle input      [0, 1]
    brake,           # brake input         [0, 1]
    waypoint_dx,     # next waypoint Δx    (m)
    waypoint_dy,     # next waypoint Δy    (m)
]  # dim = 8
```

### Output (predicted next state)

```python
next_state = [
    delta_x,         # position change x   (m)
    delta_y,         # position change y   (m)
    delta_yaw,       # heading change      (rad)
    velocity_x,      # next longitudinal speed
    velocity_y,      # next lateral speed
]  # dim = 5
```

### Architecture

```
Input (seq_len=20, features=8)
    │
    ▼
LSTM(input=8, hidden=256, layers=2, dropout=0.1)
    │
    ▼  last hidden state
Linear(256 → 128) → ReLU
    │
    ▼
Linear(128 → 5)   ← predicted Δstate
```

---

## MPC Formulation

### Cost Function

$$J = \sum_{k=0}^{H-1} \left[ w_1 \|p_k - p_k^{ref}\|^2 + w_2 \|\dot{e}_{heading}\|^2 + w_3 \|u_k\|^2 + w_4 \|\Delta u_k\|^2 \right]$$

| Term | Weight | Description |
|---|---|---|
| Position tracking | `w1 = 10.0` | Distance to reference trajectory |
| Heading error rate | `w2 = 5.0` | Rate of heading deviation |
| Control effort | `w3 = 0.1` | Magnitude of control inputs |
| Control smoothness | `w4 = 1.0` | Change between consecutive inputs |

### Constraints

```
-1.0  ≤  steering  ≤  1.0
 0.0  ≤  throttle  ≤  1.0
 0.0  ≤  brake     ≤  1.0
 speed ≤ speed_limit (configurable)
```

### Hyperparameters

| Parameter | Value | Description |
|---|---|---|
| Horizon `H` | 10 | Prediction steps |
| `dt` | 0.1 s | Timestep |
| Sequence length | 20 | LSTM input window |
| Solver | SLSQP / CasADi | Optimization backend |

---

## Integration with SAC Pipeline

The MPC-LSTM controller can operate in two modes:

### Mode 1 — Standalone Controller
```
CarlaRLEnv → MPCLSTMAgent → CARLA actions
```

### Mode 2 — Residual Control (SAC + MPC)
```
CarlaRLEnv → SAC Policy (high-level) → reference trajectory
                                              │
                                              ▼
                                       MPCLSTMAgent → CARLA actions
```

---

## Training the LSTM Dynamics Model

### Step 1 — Collect Data
```bash
cd RL_Agent_SAC
python MPC_LSTM/data_collector.py \
    --episodes 500 \
    --config config/sac_config.yaml \
    --output data/dynamics_data.pkl
```

### Step 2 — Train LSTM
```bash
python MPC_LSTM/trainer.py \
    --data data/dynamics_data.pkl \
    --epochs 100 \
    --batch-size 64 \
    --hidden-size 256 \
    --seq-len 20 \
    --output checkpoints/lstm_dynamics.pt
```

### Step 3 — Run MPC-LSTM Controller
```bash
python MPC_LSTM/mpc_lstm_agent.py \
    --model checkpoints/lstm_dynamics.pt \
    --config config/sac_config.yaml \
    --horizon 10
```

---

## Directory Structure

```
MPC_LSTM/
├── README.md                  ← This file
├── lstm_model.py              ← LSTM dynamics model definition
├── mpc_controller.py          ← MPC optimizer (SLSQP / CasADi)
├── mpc_lstm_agent.py          ← Combined agent interface
├── data_collector.py          ← Rollout data collection from CARLA
├── trainer.py                 ← LSTM training loop
├── config/
│   └── mpc_config.yaml        ← MPC & LSTM hyperparameters
├── checkpoints/
│   └── lstm_dynamics.pt       ← Trained LSTM weights
└── data/
    └── dynamics_data.pkl      ← Collected training data
```

---

## Configuration (`mpc_config.yaml`)

```yaml
lstm:
  hidden_size: 256
  num_layers: 2
  dropout: 0.1
  sequence_length: 20
  input_dim: 8
  output_dim: 5
  learning_rate: 0.001
  batch_size: 64
  epochs: 100

mpc:
  horizon: 10
  dt: 0.1
  solver: slsqp            # Options: slsqp, casadi
  max_iterations: 100
  weights:
    position: 10.0
    heading_rate: 5.0
    control_effort: 0.1
    control_smoothness: 1.0
  constraints:
    steering: [-1.0, 1.0]
    throttle: [0.0, 1.0]
    brake: [0.0, 1.0]
    max_speed: 13.9          # ~50 km/h in m/s
```

---

## Dependencies

```
torch>=2.0
numpy>=1.24
scipy>=1.10          # for SLSQP solver
casadi>=3.6          # optional: for faster MPC solving
carla==0.9.16
```

---

## References

- [Model Predictive Control — Rawlings et al. (2017)](https://doi.org/10.1002/aic.16235)
- [Learning-based MPC with LSTM — Becker et al. (2019)](https://arxiv.org/abs/1910.04099)
- [CARLA Simulator](https://carla.org)
- [Stable-Baselines3 SAC](https://stable-baselines3.readthedocs.io)

---

*Part of the [Carla-fullself-driving](https://github.com/Telotubbies/Carla-fullself-driving) project.*
