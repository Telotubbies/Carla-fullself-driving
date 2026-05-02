#!/usr/bin/env python3
"""Load trained SAC model and run inference with visualization.

Usage:
    venv/bin/python scripts/inference_show.py \
        --checkpoint checkpoints/gt \
        --episodes 5 \
        --record

The script loads the trained policy from the RLlib checkpoint and runs
inference episodes in CARLA, optionally recording videos.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np
import yaml

import ray
from ray.rllib.algorithms.sac import SACConfig
from ray.tune.registry import register_env

from src.carla_gym_env import CarlaEnv
from src.env_wrappers import CurriculumEnvWrapper, GTStateWrapper
from src.env_wrappers.episode_recorder import EpisodeRecorderWrapper
from src.gt_state import DEFAULT_SCHEMA, StateBuilder, NoiseConfig


def load_config(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def env_factory(env_config: dict):
    """Create env with same wrappers as training."""
    cfg = load_config(env_config.get("config_path", "config/gt_state.yaml"))
    
    # Build schema + noise
    p = cfg["perception"]
    b = cfg["bev"]
    schema = DEFAULT_SCHEMA
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
        "host": env_config.get("host", "localhost"),
        "port": int(env_config.get("port", 2000)),
        "timeout": 30.0,
        "map": env_config.get("map", "Town10HD_Opt"),
        "delta_seconds": 0.05,
        "max_episode_steps": env_config.get("max_episode_steps", 1000),
        "use_camera": False,
        "use_fixed_spawn": False,
        "fixed_spawn_indices": [],
        "disable_perception_sensors": True,
        "sensor_config": {},
    }
    
    env = CarlaEnv(carla_cfg)
    env = CurriculumEnvWrapper(env)
    env = GTStateWrapper(
        env, schema=schema, noise=noise,
        jerk_penalty_weight=0.0,  # no penalty during inference
        follow_spectator=True,
        spectator_every_n_steps=2,
    )
    
    if env_config.get("record", False):
        env = EpisodeRecorderWrapper(
            env,
            output_dir=env_config.get("output_dir", "artifacts/inference"),
            fps=20,
            enable=True,
        )
    
    return env


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", required=True, help="Path to RLlib checkpoint dir")
    ap.add_argument("--episodes", type=int, default=5)
    ap.add_argument("--steps", type=int, default=1000)
    ap.add_argument("--record", action="store_true", help="Record episode videos")
    ap.add_argument("--config", default="config/gt_state.yaml")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--render", action="store_true", default=True, help="CARLA spectator follows")
    ap.add_argument("--deterministic", action="store_true", help="Use deterministic actions (mean only)")
    args = ap.parse_args()
    
    # Register env
    register_env("carla_gt_env", env_factory)
    
    # Initialize Ray
    ray.init(ignore_reinit_error=True)
    
    # Build config matching training
    cfg = load_config(args.config)
    
    sac = (
        SACConfig()
        .environment(
            env="carla_gt_env",
            env_config={
                "config_path": args.config,
                "host": args.host,
                "port": args.port,
                "record": args.record,
                "output_dir": "artifacts/inference",
                "max_episode_steps": args.steps,
            },
            action_space=None,
            observation_space=None,
        )
        .api_stack(
            enable_rl_module_and_learner=False,
            enable_env_runner_and_connector_v2=False,
        )
        .training(
            replay_buffer_config={
                "type": "MultiAgentPrioritizedReplayBuffer",
                "capacity": 1000,
                "prioritized_replay_alpha": 0.6,
                "prioritized_replay_beta": 0.4,
            },
            _disable_preprocessor_api=True,
        )
        .resources(num_gpus=0)
        .debugging(seed=42)
    )
    
    # Build algorithm and restore checkpoint
    print(f"[inference] Loading checkpoint from: {args.checkpoint}")
    algo = sac.build()
    algo.restore(args.checkpoint)
    print("[inference] Checkpoint loaded successfully!")
    
    # Run inference episodes
    total_reward = 0.0
    total_steps = 0
    
    for ep in range(args.episodes):
        print(f"\n[inference] Episode {ep+1}/{args.episodes}")
        
        # Get env from algorithm
        env_runner = algo.env_runner_group.local_env_runner
        obs, info = env_runner.env.reset()
        
        episode_reward = 0.0
        episode_steps = 0
        done = False
        
        while not done and episode_steps < args.steps:
            # Compute action
            if args.deterministic:
                # Use policy's mean (no sampling)
                action = algo.compute_single_action(obs, explore=False)
            else:
                action = algo.compute_single_action(obs, explore=True)
            
            # Step environment
            obs, reward, terminated, truncated, info = env_runner.env.step(action)
            episode_reward += reward
            episode_steps += 1
            done = terminated or truncated
            
            # Slow down for visualization
            if args.render:
                time.sleep(0.02)
        
        total_reward += episode_reward
        total_steps += episode_steps
        
        print(f"  Episode {ep+1}: reward={episode_reward:+.2f}, steps={episode_steps}")
        if info.get("collision", False):
            print("  -> COLLISION")
        elif info.get("off_road", False):
            print("  -> OFF ROAD")
        else:
            print("  -> SUCCESS (no collision)")
    
    avg_reward = total_reward / args.episodes
    avg_steps = total_steps / args.episodes
    
    print(f"\n[inference] Summary ({args.episodes} episodes):")
    print(f"  Average reward: {avg_reward:+.2f}")
    print(f"  Average steps:  {avg_steps:.1f}")
    
    if args.record:
        print(f"\n  Videos saved to: artifacts/inference/")
    
    algo.stop()
    ray.shutdown()
    print("[inference] Done!")


if __name__ == "__main__":
    main()
