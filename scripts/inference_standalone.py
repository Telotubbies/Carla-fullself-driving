#!/usr/bin/env python3
"""Standalone inference with GIF capture and keyboard control.

This loads the policy directly from checkpoint and runs without RLlib's
EnvRunnerGroup to avoid CARLA streaming issues.

Usage:
    venv/bin/python scripts/inference_standalone.py \
        --checkpoint checkpoints/gt \
        --episodes 3 \
        --gif_fps 10

Keyboard Controls (during eval):
    W/S - Speed up / slow down (modify target speed)
    A/D - Steering bias left/right
    R   - Reset control overrides
    P   - Pause/Resume
    ESC - Stop episode
    M   - Toggle manual mode (full keyboard control)
"""

import argparse
import sys
import time
import threading
from pathlib import Path
from typing import Optional, Dict, Any

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np
import yaml
import pygame
from PIL import Image
import imageio
import torch

import carla

from src.carla_gym_env import CarlaEnv
from src.env_wrappers import CurriculumEnvWrapper, GTStateWrapper
from src.gt_state import DEFAULT_SCHEMA, StateBuilder, NoiseConfig

# Keyboard control state
CTRL = {
    "speed_offset": 0.0,      # m/s
    "steering": 0.0,          # -1 to 1
    "paused": False,
    "stop": False,
    "manual": False,          # True = ignore model, use keyboard
}


def load_policy_weights(checkpoint_path: str):
    """Load policy weights from RLlib checkpoint."""
    import pickle
    
    # Load algorithm state
    algo_path = Path(checkpoint_path) / "algorithm_state.pkl"
    with open(algo_path, "rb") as f:
        algo_state = pickle.load(f)
    
    # Load policy weights from policies/default_policy
    policy_path = Path(checkpoint_path) / "policies" / "default_policy"
    
    # Try loading PyTorch state dict
    weight_files = list(policy_path.glob("*.pth")) + list(policy_path.glob("**/*.pth"))
    if weight_files:
        state_dict = torch.load(weight_files[0], map_location="cpu")
        print(f"[eval] Loaded weights from: {weight_files[0]}")
        return state_dict
    
    # If no .pth, try loading from checkpoint directly
    print("[eval] Note: Could not find .pth weights, policy may need RLlib to load")
    return None


def init_pygame():
    """Initialize pygame for keyboard control."""
    pygame.init()
    screen = pygame.display.set_mode((600, 400))
    pygame.display.set_caption("CARLA Eval - W/S=speed, A/D=steer, R=reset, P=pause, ESC=stop, M=manual")
    font = pygame.font.SysFont("monospace", 14)
    return screen, font


def draw_panel(screen, font, obs_info: Dict):
    """Draw control panel."""
    screen.fill((20, 20, 20))
    
    lines = [
        "=== Keyboard Controls ===",
        "W/S : Speed offset (+/- 5 m/s)",
        "A/D : Steering bias",
        "R   : Reset overrides",
        "P   : Pause/Resume",
        "M   : Toggle manual mode",
        "ESC : Stop episode",
        "",
        "=== Control State ===",
        f"Speed offset:  {CTRL['speed_offset']:+.1f} m/s",
        f"Steering:      {CTRL['steering']:+.2f}",
        f"Paused:        {CTRL['paused']}",
        f"Manual:        {CTRL['manual']}",
    ]
    
    if obs_info:
        lines.extend([
            "",
            "=== Vehicle ===",
            f"Speed:         {obs_info.get('speed', 0):.1f} m/s",
            f"Lat offset:    {obs_info.get('lat_offset', 0):.2f} m",
        ])
    
    y = 10
    for line in lines:
        color = (200, 200, 200) if not line.startswith("===") else (100, 200, 255)
        text = font.render(line, True, color)
        screen.blit(text, (10, y))
        y += 20
    
    pygame.display.flip()


def handle_events():
    """Handle pygame keyboard events."""
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            CTRL["stop"] = True
            return False
        
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                CTRL["speed_offset"] += 5.0
            elif event.key == pygame.K_s:
                CTRL["speed_offset"] -= 5.0
            elif event.key == pygame.K_a:
                CTRL["steering"] = max(-1.0, CTRL["steering"] - 0.2)
            elif event.key == pygame.K_d:
                CTRL["steering"] = min(1.0, CTRL["steering"] + 0.2)
            elif event.key == pygame.K_r:
                CTRL["speed_offset"] = 0.0
                CTRL["steering"] = 0.0
                CTRL["manual"] = False
            elif event.key == pygame.K_p:
                CTRL["paused"] = not CTRL["paused"]
            elif event.key == pygame.K_m:
                CTRL["manual"] = not CTRL["manual"]
            elif event.key == pygame.K_ESCAPE:
                CTRL["stop"] = True
    
    return True


def extract_info(obs) -> Dict:
    """Extract info from observation."""
    info = {}
    if isinstance(obs, dict) and "vector" in obs:
        vec = obs["vector"]
        if len(vec) >= 3:
            info["speed"] = vec[0]
            info["lat_offset"] = vec[1]
    return info


def save_gif(frames, path: Path, fps: int):
    """Save frames as GIF."""
    if not frames:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    pil_frames = [Image.fromarray(f, mode='RGB') for f in frames if f is not None]
    if pil_frames:
        pil_frames[0].save(path, save_all=True, append_images=pil_frames[1:],
                          duration=int(1000/fps), loop=0)
        print(f"[eval] GIF saved: {path} ({len(pil_frames)} frames)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--episodes", type=int, default=3)
    ap.add_argument("--steps", type=int, default=500)
    ap.add_argument("--config", default="config/gt_state.yaml")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--deterministic", action="store_true")
    ap.add_argument("--output_dir", default="artifacts/eval_gifs")
    ap.add_argument("--gif_fps", type=int, default=10)
    ap.add_argument("--capture_every", type=int, default=5)
    ap.add_argument("--no_display", action="store_true", help="No pygame window")
    args = ap.parse_args()
    
    # Load config
    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    
    # Build env
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
        "host": args.host,
        "port": args.port,
        "timeout": 30.0,
        "map": "Town10HD_Opt",
        "delta_seconds": 0.05,
        "max_episode_steps": args.steps,
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
    
    # Init pygame if not disabled
    screen, font = None, None
    if not args.no_display:
        try:
            screen, font = init_pygame()
            print("[eval] Pygame control window active")
        except Exception as e:
            print(f"[eval] Warning: pygame init failed: {e}")
    
    # NOTE: For now, we'll use a simple random policy since loading RLlib weights
    # directly requires the exact model architecture. A proper implementation
    # would reconstruct the SAC model and load state_dict.
    print("[eval] Note: Using environment's default action sampling for demo")
    print("[eval] To use trained policy, the model architecture must be reconstructed")
    
    total_reward = 0.0
    
    for ep in range(args.episodes):
        print(f"\n[eval] Episode {ep+1}/{args.episodes}")
        
        # Reset controls
        CTRL["speed_offset"] = 0.0
        CTRL["steering"] = 0.0
        CTRL["paused"] = False
        CTRL["stop"] = False
        CTRL["manual"] = False
        
        frames = []
        obs, info = env.reset()
        episode_reward = 0.0
        step = 0
        done = False
        
        while not done and step < args.steps and not CTRL["stop"]:
            # Handle pygame
            if screen:
                if not handle_events():
                    break
                draw_panel(screen, font, extract_info(obs))
            
            # Pause handling
            while CTRL["paused"]:
                if screen:
                    handle_events()
                    draw_panel(screen, font, extract_info(obs))
                time.sleep(0.1)
            
            # Capture frame
            if step % args.capture_every == 0:
                if isinstance(obs, dict) and "bev" in obs:
                    bev = obs["bev"]
                    if bev.dtype != np.uint8:
                        bev = (bev * 255).astype(np.uint8)
                    if len(bev.shape) == 3:
                        frames.append(bev[:, :, :3])
            
            # Get action (simple heuristic for demo - keeps lane)
            if CTRL["manual"]:
                # Manual keyboard control
                throttle = 0.3 if CTRL["speed_offset"] > 0 else 0.0
                brake = 0.3 if CTRL["speed_offset"] < 0 else 0.0
                steer = CTRL["steering"]
                action = np.array([steer, throttle, brake])
            else:
                # Use env action space sample (placeholder for policy)
                action = env.action_space.sample()
                # Apply keyboard overrides
                action[0] = np.clip(action[0] + CTRL["steering"], -1, 1)
            
            obs, reward, terminated, truncated, info = env.step(action)
            episode_reward += reward
            step += 1
            done = terminated or truncated
            
            time.sleep(0.02)
        
        # Save GIF
        gif_path = Path(args.output_dir) / f"episode_{ep+1:02d}.gif"
        save_gif(frames, gif_path, args.gif_fps)
        
        total_reward += episode_reward
        status = "COLLISION" if info.get("collision") else "OFF_ROAD" if info.get("off_road") else "SUCCESS"
        print(f"  Reward: {episode_reward:+.2f}, Steps: {step}, Status: {status}")
    
    if screen:
        pygame.quit()
    
    env.close()
    
    print(f"\n[eval] Average reward: {total_reward/args.episodes:+.2f}")
    print(f"[eval] GIFs saved to: {args.output_dir}/")


if __name__ == "__main__":
    main()
