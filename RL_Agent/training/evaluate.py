#!/usr/bin/env python3
"""
Evaluation Script for Trained RL Agent
Test the agent's performance in CARLA environment
"""

import argparse
import os
import sys
import yaml
import numpy as np
import torch
from pathlib import Path
import time

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv

from carla_env.carla_rl_env import CarlaRLEnv
from models.custom_policy import VisionActorCriticPolicy
from utils.gpu_utils import get_device


def load_config(config_path: str) -> dict:
    """Load configuration from YAML file"""
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config


def make_env(config: dict, seed: int = 0):
    """Create environment"""
    def _init():
        env = CarlaRLEnv(config)
        env = Monitor(env, filename=None, allow_early_resets=True)
        env.reset(seed=seed)
        return env
    return _init


def evaluate_agent(model_path: str, config_path: str, num_episodes: int = 10):
    """
    Evaluate trained agent
    
    Args:
        model_path: Path to trained model
        config_path: Path to configuration file
        num_episodes: Number of episodes to evaluate
    """
    print("=" * 60)
    print("Evaluating RL Agent")
    print("=" * 60)
    
    # Load configuration
    config = load_config(config_path)
    device = get_device(config)
    
    # Create environment
    print("\n[1/3] Creating environment...")
    env = DummyVecEnv([make_env(config, seed=42)])
    print("✅ Environment created")
    
    # Load model
    print(f"\n[2/3] Loading model from {model_path}...")
    model = PPO.load(model_path, env=env, device=device)
    print("✅ Model loaded")
    
    # Evaluate
    print(f"\n[3/3] Running evaluation for {num_episodes} episodes...")
    print("=" * 60)
    
    episode_rewards = []
    episode_lengths = []
    collisions = []
    
    for episode in range(num_episodes):
        obs, info = env.reset()
        done = False
        episode_reward = 0.0
        episode_length = 0
        collision_occurred = False
        
        print(f"\nEpisode {episode + 1}/{num_episodes}")
        
        while not done:
            # Get action from policy
            action, _ = model.predict(obs, deterministic=True)
            
            # Step environment
            obs, reward, done, truncated, info = env.step(action)
            
            episode_reward += reward[0]
            episode_length += 1
            
            if info[0].get('collision', False):
                collision_occurred = True
            
            # Print progress
            if episode_length % 100 == 0:
                speed = info[0].get('speed', 0.0)
                print(f"  Step {episode_length}: Reward={reward[0]:.2f}, Speed={speed:.1f} km/h")
        
        episode_rewards.append(episode_reward)
        episode_lengths.append(episode_length)
        collisions.append(1 if collision_occurred else 0)
        
        print(f"  Episode completed:")
        print(f"    Total Reward: {episode_reward:.2f}")
        print(f"    Length: {episode_length} steps")
        print(f"    Collision: {'Yes' if collision_occurred else 'No'}")
    
    # Print statistics
    print("\n" + "=" * 60)
    print("Evaluation Results")
    print("=" * 60)
    print(f"Episodes: {num_episodes}")
    print(f"Mean Reward: {np.mean(episode_rewards):.2f} ± {np.std(episode_rewards):.2f}")
    print(f"Mean Length: {np.mean(episode_lengths):.1f} ± {np.std(episode_lengths):.1f}")
    print(f"Collision Rate: {np.mean(collisions) * 100:.1f}%")
    print(f"Max Reward: {np.max(episode_rewards):.2f}")
    print(f"Min Reward: {np.min(episode_rewards):.2f}")
    print("=" * 60)
    
    # Cleanup
    env.close()
    print("\n✅ Evaluation completed")


def main():
    parser = argparse.ArgumentParser(description='Evaluate Trained RL Agent')
    parser.add_argument(
        '--model',
        type=str,
        required=True,
        help='Path to trained model checkpoint'
    )
    parser.add_argument(
        '--config',
        type=str,
        default='config/phase1_config.yaml',
        help='Path to configuration file'
    )
    parser.add_argument(
        '--episodes',
        type=int,
        default=10,
        help='Number of episodes to evaluate'
    )
    
    args = parser.parse_args()
    
    # Resolve paths
    base_dir = Path(__file__).parent.parent
    model_path = args.model if os.path.isabs(args.model) else os.path.join(base_dir, args.model)
    config_path = args.config if os.path.isabs(args.config) else os.path.join(base_dir, args.config)
    
    if not os.path.exists(model_path):
        print(f"❌ Model not found: {model_path}")
        sys.exit(1)
    
    if not os.path.exists(config_path):
        print(f"❌ Config not found: {config_path}")
        sys.exit(1)
    
    evaluate_agent(model_path, config_path, args.episodes)


if __name__ == '__main__':
    main()

