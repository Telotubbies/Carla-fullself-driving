#!/usr/bin/env python3
"""Eval trained SAC model with GIF recording.

Usage:
    venv/bin/python scripts/inference_gif.py \
        --checkpoint checkpoints/gt \
        --episodes 3 \
        --gif_fps 10

Captures BEV frames and saves as animated GIF.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Optional, Dict, Any

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np
import yaml
from PIL import Image
import imageio

import ray
from ray.rllib.algorithms.sac import SACConfig
from ray.tune.registry import register_env

from src.carla_gym_env import CarlaEnv
from src.env_wrappers import CurriculumEnvWrapper, GTStateWrapper
from src.gt_state import DEFAULT_SCHEMA, StateBuilder, NoiseConfig


def load_config(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def env_factory(env_config: dict):
    """Create env with same wrappers as training."""
    cfg = load_config(env_config.get("config_path", "config/gt_state.yaml"))
    
    p = cfg["perception"]
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
        jerk_penalty_weight=0.0,
        follow_spectator=True,
        spectator_every_n_steps=2,
    )
    
    return env


def capture_bev_frame(obs: Dict[str, Any]) -> Optional[np.ndarray]:
    """Extract BEV image from observation for GIF."""
    if isinstance(obs, dict) and "bev" in obs:
        bev = obs["bev"]
        # Ensure uint8 RGB format
        if bev.dtype != np.uint8:
            bev = (bev * 255).astype(np.uint8)
        if len(bev.shape) == 3 and bev.shape[2] in [3, 5]:  # RGB or multi-channel
            # Take first 3 channels as RGB
            return bev[:, :, :3]
    return None


def save_gif(frames: list, output_path: Path, fps: int = 10):
    """Save collected frames as animated GIF."""
    if not frames:
        print(f"[eval] No frames to save for {output_path}")
        return
    
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    pil_frames = []
    for frame in frames:
        if frame is not None:
            pil_frames.append(Image.fromarray(frame, mode='RGB'))
    
    if pil_frames:
        pil_frames[0].save(
            output_path,
            save_all=True,
            append_images=pil_frames[1:],
            duration=int(1000 / fps),
            loop=0
        )
        print(f"[eval] GIF saved: {output_path} ({len(pil_frames)} frames, {fps} fps)")
    else:
        print(f"[eval] No valid frames for {output_path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--episodes", type=int, default=3)
    ap.add_argument("--steps", type=int, default=1000)
    ap.add_argument("--config", default="config/gt_state.yaml")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--deterministic", action="store_true")
    ap.add_argument("--output_dir", default="artifacts/eval_gifs")
    ap.add_argument("--gif_fps", type=int, default=10)
    ap.add_argument("--capture_every", type=int, default=5)
    args = ap.parse_args()
    
    register_env("carla_gt_env", env_factory)
    ray.init(ignore_reinit_error=True)
    
    cfg = load_config(args.config)
    
    sac = (
        SACConfig()
        .api_stack(
            enable_rl_module_and_learner=False,
            enable_env_runner_and_connector_v2=False,
        )
        .environment(
            env="carla_gt_env",
            env_config={
                "config_path": args.config,
                "host": args.host,
                "port": args.port,
                "max_episode_steps": args.steps,
            },
        )
        .framework("torch")
        .training(
            replay_buffer_config={
                "type": "MultiAgentPrioritizedReplayBuffer",
                "capacity": 10000,
                "prioritized_replay_alpha": 0.6,
                "prioritized_replay_beta": 0.4,
                "prioritized_replay_eps": 1e-6,
            },
        )
        .resources(num_gpus=0)
        .debugging(seed=42)
    )
    
    print(f"[eval] Loading checkpoint from: {args.checkpoint}")
    algo = sac.build()
    algo.restore(args.checkpoint)
    print("[eval] Checkpoint loaded successfully!")
    
    total_reward = 0.0
    total_steps = 0
    
    for ep in range(args.episodes):
        print(f"\n[eval] Episode {ep+1}/{args.episodes}")
        
        episode_frames = []
        
        # Get env from algorithm
        env_runner = algo.env_runner_group.local_env_runner
        obs, info = env_runner.env.reset()
        
        episode_reward = 0.0
        episode_steps = 0
        done = False
        step_count = 0
        
        while not done and episode_steps < args.steps:
            # Capture frame
            if step_count % args.capture_every == 0:
                frame = capture_bev_frame(obs)
                if frame is not None:
                    episode_frames.append(frame)
            
            # Compute action
            if args.deterministic:
                action = algo.compute_single_action(obs, explore=False)
            else:
                action = algo.compute_single_action(obs, explore=True)
            
            # Step environment
            obs, reward, terminated, truncated, info = env_runner.env.step(action)
            episode_reward += reward
            episode_steps += 1
            done = terminated or truncated
            step_count += 1
            
            time.sleep(0.02)  # Slow down for visualization
        
        # Save episode GIF
        gif_path = Path(args.output_dir) / f"episode_{ep+1:02d}.gif"
        save_gif(episode_frames, gif_path, fps=args.gif_fps)
        
        total_reward += episode_reward
        total_steps += episode_steps
        
        print(f"  Episode {ep+1}: reward={episode_reward:+.2f}, steps={episode_steps}")
        status = "COLLISION" if info.get("collision") else "OFF_ROAD" if info.get("off_road") else "SUCCESS"
        print(f"  -> {status}")
    
    avg_reward = total_reward / args.episodes
    avg_steps = total_steps / args.episodes
    
    print(f"\n[eval] Summary ({args.episodes} episodes):")
    print(f"  Average reward: {avg_reward:+.2f}")
    print(f"  Average steps:  {avg_steps:.1f}")
    print(f"  GIFs saved to:  {args.output_dir}/")
    
    algo.stop()
    ray.shutdown()
    print("[eval] Done!")


if __name__ == "__main__":
    main()
