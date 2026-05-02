#!/usr/bin/env python3
"""Deploy trained SAC model for actual autonomous driving in CARLA.

This script loads the trained checkpoint and runs the vehicle autonomously
using the learned policy. Unlike training, this uses the policy deterministically
(exploitation only) for best performance.

Usage:
    venv/bin/python deploy_model.py \
        --checkpoint checkpoints/gt \
        --episodes 5 \
        --steps 2000 \
        --record

The vehicle will drive autonomously in CARLA using the trained model.
"""

from __future__ import annotations

import argparse
import sys
import time
import math
from pathlib import Path

ROOT = Path(__file__).parent.resolve()
sys.path.insert(0, str(ROOT))

import numpy as np
import yaml
import gymnasium as gym

import carla  # type: ignore

from src.carla_gym_env import CarlaEnv
from src.env_wrappers import GTStateWrapper
from src.env_wrappers.episode_recorder import EpisodeRecorderWrapper
from src.gt_state import DEFAULT_SCHEMA, NoiseConfig


def load_config(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


class DeployedPolicy:
    """Simple deployment wrapper that loads RLlib checkpoint and computes actions."""
    
    def __init__(self, checkpoint_path: str, config_path: str = "config/gt_state.yaml"):
        self.checkpoint_path = checkpoint_path
        self.cfg = load_config(config_path)
        
        # Import RLlib here to avoid heavy import if not needed
        import ray
        from ray.rllib.algorithms.sac import SAC
        
        # Initialize Ray
        if not ray.is_initialized():
            ray.init(ignore_reinit_error=True, log_to_driver=False)
        
        # Load checkpoint directly - config is embedded in checkpoint
        print(f"[deploy] Loading checkpoint from: {checkpoint_path}")
        from ray.rllib.algorithms.algorithm import Algorithm
        
        self.algo = Algorithm.from_checkpoint(checkpoint_path)
        print("[deploy] Model loaded successfully from checkpoint!")
    
    def compute_action(self, observation: dict, deterministic: bool = True) -> np.ndarray:
        """Compute action from observation."""
        # Handle Dict observation space
        if isinstance(observation, dict):
            # Convert to the format expected by RLlib
            obs = {
                "vector": np.array(observation["vector"], dtype=np.float32),
                "bev": np.array(observation["bev"], dtype=np.float32),
            }
        else:
            obs = observation
        
        # Compute action using RLlib policy
        action = self.algo.compute_single_action(
            obs,
            explore=not deterministic,
        )
        return np.array(action)
    
    def stop(self):
        """Clean up resources."""
        self.algo.stop()


def create_env(args):
    """Create CARLA environment with wrappers."""
    cfg = load_config(args.config)
    
    p = cfg["perception"]
    noise = NoiseConfig(
        latency_frames=int(p.get("latency_ms", 0) / 50),
        pos_sigma=p.get("pos_noise_std", 0.1),
        vel_sigma=0.1,
        ego_pos_sigma=0.0,
        ego_vel_sigma=0.0,
        dropout_p=p.get("dropout_p", 0.0),
        seed=None,
    )
    
    carla_cfg = {
        "host": args.host,
        "port": args.port,
        "timeout": 30.0,
        "map": cfg.get("environment", {}).get("map", "Town10HD_Opt"),
        "delta_seconds": 0.05,
        "max_episode_steps": args.steps,
        "use_camera": False,
        "use_fixed_spawn": args.fixed_spawn,
        "fixed_spawn_indices": [int(x) for x in args.spawn_indices.split(",")] if args.spawn_indices else [],
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
    
    if args.record:
        env = EpisodeRecorderWrapper(
            env,
            output_dir=args.output_dir,
            fps=20,
            enable=True,
        )
    
    return env


def main():
    ap = argparse.ArgumentParser(description="Deploy trained SAC model in CARLA")
    ap.add_argument("--checkpoint", required=True, help="Path to RLlib checkpoint")
    ap.add_argument("--config", default="config/gt_state.yaml")
    ap.add_argument("--episodes", type=int, default=5)
    ap.add_argument("--steps", type=int, default=2000)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--record", action="store_true", help="Record episode videos")
    ap.add_argument("--output-dir", default="artifacts/deploy")
    ap.add_argument("--deterministic", action="store_true", default=True,
                    help="Use deterministic policy (no exploration)")
    ap.add_argument("--fixed-spawn", action="store_true", default=False,
                    help="Use fixed spawn points instead of random")
    ap.add_argument("--spawn-indices", default="",
                    help="Comma-separated spawn indices (e.g., '0,1,2')")
    args = ap.parse_args()
    
    print("=" * 60)
    print("🚗 CARLA Autonomous Driving - Model Deployment")
    print("=" * 60)
    
    # Load trained policy
    policy = DeployedPolicy(args.checkpoint, args.config)
    
    # Create environment
    print("\n[deploy] Creating CARLA environment...")
    env = create_env(args)
    
    print(f"\n[deploy] Running {args.episodes} episodes...")
    print(f"  Deterministic: {args.deterministic}")
    print(f"  Fixed spawn: {args.fixed_spawn}")
    print(f"  Recording: {args.record}")
    
    total_reward = 0.0
    total_steps = 0
    collisions = 0
    successes = 0
    
    for ep in range(args.episodes):
        print(f"\n{'='*60}")
        print(f"🎬 Episode {ep + 1}/{args.episodes}")
        print("="*60)
        
        obs, info = env.reset()
        episode_reward = 0.0
        episode_steps = 0
        done = False
        
        while not done and episode_steps < args.steps:
            # Get action from trained policy
            action = policy.compute_action(obs, deterministic=args.deterministic)
            
            # Step environment
            obs, reward, terminated, truncated, info = env.step(action)
            
            episode_reward += reward
            episode_steps += 1
            done = terminated or truncated
            
            # Print status every 100 steps
            if episode_steps % 100 == 0:
                speed = info.get("speed", 0.0)
                print(f"  Step {episode_steps}: speed={speed:.1f}m/s, reward={reward:+.2f}")
        
        # Episode summary
        total_reward += episode_reward
        total_steps += episode_steps
        
        has_collision = info.get("collision", False)
        is_offroad = info.get("off_road", False)
        
        if has_collision:
            collisions += 1
            status = "💥 COLLISION"
        elif is_offroad:
            status = "🚧 OFF ROAD"
        else:
            successes += 1
            status = "✅ SUCCESS"
        
        print(f"\n📊 Episode {ep + 1} Summary:")
        print(f"  Total reward: {episode_reward:+.2f}")
        print(f"  Steps: {episode_steps}")
        print(f"  Status: {status}")
        print(f"  Final speed: {info.get('speed', 0.0):.2f} m/s")
        
        # Brief pause between episodes
        if ep < args.episodes - 1:
            print("\n⏳ Starting next episode in 3 seconds...")
            time.sleep(3)
    
    # Final summary
    print(f"\n{'='*60}")
    print("📈 DEPLOYMENT SUMMARY")
    print("="*60)
    print(f"  Episodes: {args.episodes}")
    print(f"  Successes: {successes} ({100*successes/args.episodes:.1f}%)")
    print(f"  Collisions: {collisions} ({100*collisions/args.episodes:.1f}%)")
    print(f"  Avg reward: {total_reward/args.episodes:+.2f}")
    print(f"  Avg steps: {total_steps/args.episodes:.1f}")
    
    if args.record:
        print(f"\n🎥 Videos saved to: {args.output_dir}/")
    
    # Cleanup
    env.close()
    policy.stop()
    
    print("\n✅ Deployment complete!")


if __name__ == "__main__":
    main()
