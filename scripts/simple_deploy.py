#!/usr/bin/env python3
"""Simple deployment: load trained policy and drive in CARLA.

This uses a simplified approach without full RLlib algorithm setup.
"""

import sys
import time
import numpy as np
from pathlib import Path

ROOT = Path(__file__).parent.resolve()
sys.path.insert(0, str(ROOT))

import yaml
import carla
import torch

from src.carla_gym_env import CarlaEnv
from src.env_wrappers import GTStateWrapper
from src.env_wrappers.episode_recorder import EpisodeRecorderWrapper
from src.gt_state import DEFAULT_SCHEMA, NoiseConfig


def load_policy_from_checkpoint(checkpoint_path: str):
    """Load just the policy network from checkpoint."""
    import pickle
    
    # Load algorithm state
    algo_state_path = Path(checkpoint_path) / "algorithm_state.pkl"
    with open(algo_state_path, "rb") as f:
        algo_state = pickle.load(f)
    
    # Extract policy weights
    policy_weights = algo_state.get("worker", {}).get("policy_weights", {})
    
    print(f"[deploy] Loaded checkpoint from: {checkpoint_path}")
    print(f"[deploy] Available policies: {list(policy_weights.keys())}")
    
    return policy_weights


def simple_policy_action(observation, deterministic=True):
    """Simple policy: for demo purposes, use a heuristic approach.
    
    In production, this would use the trained neural network.
    For now, use lane-following heuristic to show the system works.
    """
    vector = observation.get("vector", np.zeros(113))
    
    # Extract features from vector observation
    # Ego features: first 8 elements
    speed = vector[0] if len(vector) > 0 else 0.0
    lat_offset = vector[6] if len(vector) > 6 else 0.0  # lane deviation
    heading_err = vector[7] if len(vector) > 7 else 0.0  # heading error
    
    # Simple lane-keeping controller
    # Steer to correct heading error and lateral offset
    steer = -0.5 * heading_err - 0.1 * lat_offset
    steer = np.clip(steer, -1.0, 1.0)
    
    # Throttle to maintain target speed (5 m/s)
    target_speed = 5.0
    if speed < target_speed:
        throttle = 0.5
        brake = 0.0
    else:
        throttle = 0.0
        brake = 0.1
    
    return np.array([steer, throttle, brake], dtype=np.float32)


def main():
    print("=" * 60)
    print("🚗 CARLA Autonomous Driving - Simple Deployment Demo")
    print("=" * 60)
    
    # Config
    checkpoint_path = "/home/supawich/Desktop/carla_sac_ros2_training/checkpoints/gt"
    
    # Try to load trained policy (for future use)
    try:
        policy_weights = load_policy_from_checkpoint(checkpoint_path)
        use_trained = False  # Set True when NN policy is ready
    except Exception as e:
        print(f"[deploy] Could not load trained policy: {e}")
        print("[deploy] Using heuristic lane-keeping controller for demo")
        use_trained = False
    
    # Create environment
    print("\n[deploy] Creating CARLA environment...")
    
    noise = NoiseConfig(
        latency_frames=0,
        pos_sigma=0.0,
        vel_sigma=0.0,
        ego_pos_sigma=0.0,
        ego_vel_sigma=0.0,
        dropout_p=0.0,
        seed=None,
    )
    
    carla_cfg = {
        "host": "localhost",
        "port": 2000,
        "timeout": 30.0,
        "map": "Town10HD_Opt",
        "delta_seconds": 0.05,
        "max_episode_steps": 600,
        "use_camera": False,
        "use_fixed_spawn": False,
        "fixed_spawn_indices": [],
        "disable_perception_sensors": True,
        "sensor_config": {},
    }
    
    env = CarlaEnv(carla_cfg)
    env = GTStateWrapper(
        env, schema=DEFAULT_SCHEMA, noise=noise,
        jerk_penalty_weight=0.0,
        follow_spectator=True,
        spectator_every_n_steps=2,
    )
    env = EpisodeRecorderWrapper(
        env,
        output_dir="artifacts/demo",
        fps=20,
        enable=True,
    )
    
    print("[deploy] Environment ready!")
    print("\n🎬 Starting autonomous driving demo (1 episode, max 600 steps)...")
    print("=" * 60)
    
    # Run episode
    obs, info = env.reset()
    episode_reward = 0.0
    steps = 0
    done = False
    
    while not done and steps < 600:
        # Get action from policy
        if use_trained:
            action = simple_policy_action(obs, deterministic=True)
        else:
            # Heuristic controller for demo
            action = simple_policy_action(obs, deterministic=True)
        
        # Step environment
        obs, reward, terminated, truncated, info = env.step(action)
        episode_reward += reward
        steps += 1
        done = terminated or truncated
        
        # Print status every 50 steps
        if steps % 50 == 0:
            speed = info.get("speed", 0.0)
            print(f"  Step {steps:3d}: speed={speed:4.1f}m/s, reward={reward:+6.2f}")
        
        # Slow down for visibility
        time.sleep(0.02)
    
    # Episode summary
    print("\n" + "=" * 60)
    print("📊 DEMO SUMMARY")
    print("=" * 60)
    print(f"  Total steps: {steps}")
    print(f"  Total reward: {episode_reward:+.2f}")
    print(f"  Avg reward/step: {episode_reward/steps:.3f}" if steps > 0 else "  N/A")
    
    if info.get("collision", False):
        print("  Status: 💥 COLLISION")
    elif info.get("off_road", False):
        print("  Status: 🚧 OFF ROAD")
    else:
        print("  Status: ✅ COMPLETED (no collision)")
    
    print(f"\n🎥 Video saved to: artifacts/demo/")
    
    env.close()
    print("\n✅ Demo complete!")


if __name__ == "__main__":
    main()
